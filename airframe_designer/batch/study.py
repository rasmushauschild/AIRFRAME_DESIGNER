"""Optimisation studies driven by closed-loop simulations.

A study spec (JSON):
{
  "name": "atlas08_hover_tilt",
  "airframe": "airframes/atlas_08.json",
  "scenario": "scenarios/takeoff_hover_land.json",      # or "scenarios": [...] (all are run per candidate)
  "variables": [{"path": "rotors[0:8].tilt_deg", "range": [10, 50]},
                {"path": "hover_pitch_deg", "range": [0, 40]},
                {"path": "px4.MC_PITCHRATE_P", "range": [0.05, 0.3]}],
  "objective": "phases.hold.pos_std_xy + 0.05 * phases.hold.pitch_rms_deg + 0.02 * metrics.energy_wh",
  "maximize": false,
  "constraints": ["not metrics.crashed", "ok", "phases.takeoff.time_to_alt < 15"],
  "penalty": 1000,
  "algorithm": {"name": "cmaes", "budget": 40, "population": 8, "sigma": 0.3, "seed": 1},
  "workers": 4,
  "sim": {"rate": 250, "substeps": 2, "noise": true, "speed": 0, "timeout_wall": 600}
}
With several scenarios, expressions see them under ``scenarios.<name>`` and the top level is the first one.
Results go to results/<name>/: trials.jsonl (one line per evaluation), best.json, best_airframe.json, study.json.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable

import numpy as np

from ..geometry.airframe import Airframe
from ..geometry.paths import apply_variables, get_path
from ..sim.scenario import load_scenario
from .objective import score_result
from .optimizers import make_optimizer
from .runner import run_many

PROJECT_DIR = Path(__file__).resolve().parents[2]


def load_study(spec) -> dict:
    if isinstance(spec, dict):
        return dict(spec)
    p = Path(spec)
    if not p.is_file():
        alt = PROJECT_DIR / "studies" / (p.name if p.name.endswith(".json") else p.name + ".json")
        if alt.is_file():
            p = alt
    with open(p) as f:
        d = json.load(f)
    d.setdefault("name", p.stem)
    return d


def _merge_scenarios(results: list[dict], names: list[str]) -> dict:
    """Combine per-scenario results into the dict expressions see."""
    first = dict(results[0])
    first["scenarios"] = {n: r for n, r in zip(names, results)}
    first["ok"] = all(r.get("ok", False) for r in results)
    first["failures"] = sum((r.get("failures", []) for r in results), [])
    return first


def run_study(spec, workers: int | None = None, out_dir: str | Path | None = None, log: Callable[[str], None] | None = None,
              progress: Callable[[dict], None] | None = None, resume: bool = True) -> dict:
    log = log or (lambda s: print(s, flush=True))
    st = load_study(spec)
    name = st.get("name", "study")
    out = Path(out_dir) if out_dir else PROJECT_DIR / "results" / name
    out.mkdir(parents=True, exist_ok=True)
    af = Airframe.load(st["airframe"]) if isinstance(st["airframe"], str) else Airframe.from_dict(st["airframe"])
    sc_specs = st.get("scenarios") or [st["scenario"]]
    scenarios = [load_scenario(s) for s in sc_specs]
    sc_names = [s.name for s in scenarios]
    variables = st.get("variables") or []
    if not variables:
        raise ValueError("study has no variables")
    paths = [v["path"] for v in variables]
    lo = [float(v["range"][0]) for v in variables]; hi = [float(v["range"][1]) for v in variables]
    is_int = [str(v.get("type", "float")) == "int" for v in variables]
    x0 = []
    for p in paths:
        try:
            cur = get_path(af, p)
            x0.append(float(cur[0] if isinstance(cur, list) else cur))
        except Exception:
            x0.append(float("nan"))
    x0 = None if any(np.isnan(x0)) else np.clip(np.array(x0), lo, hi)
    alg = dict(st.get("algorithm") or {"name": "random", "budget": 20})
    alg_name = alg.pop("name", "random"); budget = int(alg.pop("budget", 20)); seed = int(alg.pop("seed", 1))
    workers = int(workers or st.get("workers", 4))
    alg.setdefault("batch", workers); alg.setdefault("population", max(workers, 4)) if alg_name.startswith("cma") else None
    opt = make_optimizer(alg_name, lo, hi, budget, seed=seed, x0=x0, **alg)
    sim_kw = dict(st.get("sim") or {})
    instances = sim_kw.pop("instances", None) or st.get("instances")
    objective = st.get("objective", "metrics.energy_wh")
    constraints = list(st.get("constraints") or [])
    penalty = float(st.get("penalty", 1000.0)); maximize = bool(st.get("maximize", False))
    (out / "study.json").write_text(json.dumps(st, indent=2))
    trials_path = out / "trials.jsonl"
    cache: dict[tuple, dict] = {}
    if resume and trials_path.is_file():
        for line in trials_path.read_text().splitlines():
            try:
                t = json.loads(line)
                cache[tuple(round(float(v), 6) for v in t["x"])] = t
            except Exception:
                pass
        if cache:
            log(f"[study] resuming with {len(cache)} cached evaluations")
    trials: list[dict] = list(cache.values())
    best = min(trials, key=lambda t: t["score"]) if trials else None
    t_start = time.time()
    gen = 0
    while not opt.done:
        xs = opt.ask()
        if not xs:
            break
        gen += 1
        xs = [np.array([round(v) if ii else v for v, ii in zip(x, is_int)], float) for x in xs]
        keys = [tuple(round(float(v), 6) for v in x) for x in xs]
        todo = [(k, x) for k, x in zip(keys, xs) if k not in cache]
        tasks = []
        for k, x in todo:
            values = {p: (int(v) if ii else float(v)) for p, v, ii in zip(paths, x, is_int)}
            for si, sc in enumerate(scenarios):
                tasks.append({"id": f"g{gen}_{keys.index(k)}_{si}", "airframe": af.to_dict(), "scenario": sc.to_dict(),
                              "variables": values, "_key": k, "_si": si})
        if tasks:
            log(f"[study] generation {gen}: {len(todo)} candidates x {len(scenarios)} scenario(s) on {workers} worker(s)")
            results = run_many(tasks, workers=workers, instances=instances, **sim_kw)
            by_key: dict[tuple, list] = {}
            for t, r in zip(tasks, results):
                by_key.setdefault(t["_key"], [None] * len(scenarios))[t["_si"]] = r
            for k, x in todo:
                rs = by_key[k]
                merged = _merge_scenarios(rs, sc_names)
                sc_ = score_result(merged, objective, constraints, penalty, maximize)
                trial = {"x": [float(v) for v in x], "values": {p: float(v) for p, v in zip(paths, x)}, "score": sc_["score"],
                         "objective": sc_["objective"], "feasible": sc_["feasible"], "violations": sc_["violations"],
                         "ok": merged.get("ok"), "status": merged.get("status"), "failures": merged.get("failures", [])[:5],
                         "metrics": {n: r.get("metrics", {}) for n, r in zip(sc_names, rs)},
                         "timing": {n: r.get("timing", {}) for n, r in zip(sc_names, rs)}, "generation": gen, "t": round(time.time() - t_start, 1)}
                cache[k] = trial
                trials.append(trial)
                with open(trials_path, "a") as f:
                    f.write(json.dumps(trial) + "\n")
                if best is None or trial["score"] < best["score"]:
                    best = trial
                    cand = apply_variables(af, trial["values"])
                    cand.name = f"{af.name} [{name} best]"
                    cand.save(out / "best_airframe.json")
                    (out / "best.json").write_text(json.dumps(trial, indent=2))
                    log(f"[study] new best score {trial['score']:.4f} objective {trial['objective']} values {trial['values']}")
                if progress:
                    progress(trial)
        opt.tell(xs, [cache[k]["score"] for k in keys])
        done_n = opt.evaluated
        log(f"[study] {done_n}/{budget} evaluations, best {best['score'] if best else float('nan'):.4f}, {time.time() - t_start:.0f}s")
    summary = {"name": name, "evaluations": len(trials), "best": best, "objective": objective, "constraints": constraints,
               "variables": variables, "algorithm": alg_name, "wall_s": round(time.time() - t_start, 1), "out_dir": str(out),
               "feasible": sum(1 for t in trials if t["feasible"])}
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary
