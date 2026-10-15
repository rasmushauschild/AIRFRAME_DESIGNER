"""Command line: the interactive app, headless runs, batches, studies and static analysis.

  airframe-designer ui        [--airframe X] [--mode sitl|hitl|auto] ...     3D editor + PX4 SITL/HITL + remote
  airframe-designer run       --airframe X --scenario Y [--set path=value] [--out r.json]
  airframe-designer batch     --tasks tasks.json [--workers 4] [--out results.jsonl]
  airframe-designer study     --spec study.json [--workers 4]
  airframe-designer analyse   --airframe X [--speed-kmh 50]
  airframe-designer optimise  --airframe X --spec spec.json           (static, no PX4)
  airframe-designer export    --airframe X [--hitl] [--out file.params]
  airframe-designer paths     --airframe X                              (every variable path)
  airframe-designer scenarios                                            (list the bundled scenarios)
  airframe-designer migrate   old.json new.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]


def _parse_set(items: list[str] | None) -> dict:
    out = {}
    for it in items or []:
        if "=" not in it:
            raise SystemExit(f"--set expects path=value, got '{it}'")
        k, v = it.split("=", 1)
        try:
            out[k.strip()] = json.loads(v)
        except json.JSONDecodeError:
            out[k.strip()] = v
    return out


def cmd_run(a) -> int:
    from .batch.worker import run_once
    r = run_once(a.airframe, a.scenario, variables=_parse_set(a.set), px4_dir=a.px4_dir, instance=a.instance, speed=a.speed,
                 rate=a.rate, substeps=a.substeps, noise=not a.no_noise, seed=a.seed, timeout_wall=a.timeout,
                 log=(print if a.verbose else None), quiet=not a.verbose, extra_params=_parse_set(a.param),
                 timeseries_path=a.timeseries)
    if not a.keep_airframe:
        r.pop("airframe", None)
    text = json.dumps(r, indent=2)
    if a.out:
        Path(a.out).write_text(text)
        print(f"{'ok' if r['ok'] else 'FAILED'} {r['status']} sim {r.get('timing', {}).get('sim_s')}s wall {r.get('timing', {}).get('wall_s')}s rtf {r.get('timing', {}).get('rtf')} -> {a.out}")
        for f in r.get("failures", []):
            print("  -", f)
    else:
        print(text)
    return 0 if r["ok"] else 1


def cmd_batch(a) -> int:
    from .batch.runner import run_many
    tasks = json.loads(Path(a.tasks).read_text())
    if isinstance(tasks, dict):
        tasks = tasks.get("tasks", [])
    for k, t in enumerate(tasks):
        t.setdefault("id", f"task{k}")
        if "set" in t and "variables" not in t:
            t["variables"] = t.pop("set")
    out = open(a.out, "a") if a.out else None

    def progress(r, done, total):
        tm = r.get("timing", {})
        print(f"[{done}/{total}] {r.get('id')} {'ok' if r.get('ok') else 'FAILED'} {r.get('status')} sim {tm.get('sim_s')}s wall {tm.get('wall_s')}s rtf {tm.get('rtf')}", flush=True)
        if out:
            if not a.keep_airframe:
                r.pop("airframe", None)
            out.write(json.dumps(r) + "\n"); out.flush()

    instances = [int(x) for x in a.instances.split(",")] if a.instances else None
    results = run_many(tasks, workers=a.workers, instances=instances, progress=progress, px4_dir=a.px4_dir, speed=a.speed,
                       rate=a.rate, substeps=a.substeps, noise=not a.no_noise, timeout_wall=a.timeout)
    if out:
        out.close()
    else:
        for r in results:
            r.pop("airframe", None)
        print(json.dumps(results, indent=2))
    return 0 if all(r.get("ok") for r in results) else 1


def cmd_study(a) -> int:
    from .batch.study import run_study
    s = run_study(a.spec, workers=a.workers, out_dir=a.out)
    print(json.dumps({k: v for k, v in s.items() if k != "best"}, indent=2))
    if s.get("best"):
        b = s["best"]
        print("best:", json.dumps({"score": b["score"], "objective": b["objective"], "values": b["values"], "feasible": b["feasible"]}, indent=2))
    return 0


def cmd_analyse(a) -> int:
    from .geometry.airframe import Airframe
    from .analysis.static import analyse
    af = Airframe.load(a.airframe)
    if a.set:
        from .geometry.paths import apply_variables
        af = apply_variables(af, _parse_set(a.set))
    r = analyse(af, (a.speed_kmh / 3.6) if a.speed_kmh else None)
    r["hover_check"] = af.hover_check()
    r["validate"] = af.validate()
    print(json.dumps(r, indent=2, default=float))
    return 0


def cmd_optimise(a) -> int:
    from .geometry.airframe import Airframe
    from .analysis.geometric_optimiser import optimise
    af = Airframe.load(a.airframe)
    spec = json.loads(Path(a.spec).read_text()) if a.spec else {"variables": [], "hover_pitch": [0, 60]}
    r = optimise(af, spec, progress=lambda f, m: print(f"  {m} {f * 100:.0f}%", file=sys.stderr, flush=True))
    for m in r.get("results", []):
        m.pop("airframe", None)
    r.get("current", {}).pop("airframe", None)
    print(json.dumps(r, indent=2, default=float))
    return 0


def cmd_export(a) -> int:
    from .geometry.airframe import Airframe
    af = Airframe.load(a.airframe)
    text = af.px4_params_file(hitl=a.hitl)
    if a.out:
        Path(a.out).write_text(text); print(a.out)
    else:
        print(text)
    return 0


def cmd_paths(a) -> int:
    from .geometry.airframe import Airframe
    from .geometry.paths import list_paths, get_path
    af = Airframe.load(a.airframe)
    for p in list_paths(af):
        try:
            print(f"{p} = {get_path(af, p)}")
        except Exception:
            print(p)
    return 0


def cmd_scenarios(a) -> int:
    d = PROJECT_DIR / "scenarios"
    for p in sorted(d.glob("*.json")):
        s = json.loads(p.read_text())
        print(f"{p.name:26s} {s.get('description', '')}")
    return 0


def cmd_migrate(a) -> int:
    from .geometry.airframe import Airframe
    af = Airframe.load(a.src)
    af.save(a.dst)
    print(f"{a.src} -> {a.dst} ({af.name}: {len(af.rotors)} rotors, {len(af.wings)} wings, {len(af.legs)} legs)")
    return 0


def cmd_ui(a, argv_rest) -> int:
    from .app import main as app_main
    return app_main(argv_rest)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = argparse.ArgumentParser(prog="airframe-designer", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")

    def sim_args(p):
        p.add_argument("--px4-dir", default=None)
        p.add_argument("--speed", type=float, default=0.0, help="real-time factor, 0 = as fast as PX4 allows (default)")
        p.add_argument("--rate", type=float, default=250.0, help="sensor rate Hz")
        p.add_argument("--substeps", type=int, default=2, help="physics sub-steps per sensor step")
        p.add_argument("--no-noise", action="store_true")
        p.add_argument("--timeout", type=float, default=600.0, help="wall-clock limit per run, s")
        p.add_argument("--keep-airframe", action="store_true", help="include the full airframe dict in the output")

    p = sub.add_parser("run", help="one headless simulation"); sim_args(p)
    p.add_argument("--airframe", required=True); p.add_argument("--scenario", required=True)
    p.add_argument("--set", action="append", metavar="PATH=VALUE", help="apply a parameter path first (repeatable)")
    p.add_argument("--param", action="append", metavar="NAME=VALUE", help="extra PX4 parameter (repeatable)")
    p.add_argument("--instance", type=int, default=None); p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default=None); p.add_argument("--timeseries", default=None, help="write the sampled time series here (JSON)")
    p.add_argument("-v", "--verbose", action="store_true")

    p = sub.add_parser("batch", help="many runs in parallel from a tasks file"); sim_args(p)
    p.add_argument("--tasks", required=True); p.add_argument("--workers", type=int, default=4); p.add_argument("--out", default=None)
    p.add_argument("--instances", default=None, help="comma-separated PX4 instance numbers to use (default: free ones in 1..9)")

    p = sub.add_parser("study", help="simulation-driven optimisation study")
    p.add_argument("--spec", required=True); p.add_argument("--workers", type=int, default=None); p.add_argument("--out", default=None)

    p = sub.add_parser("analyse", help="static hover/cruise analysis"); p.add_argument("--airframe", required=True)
    p.add_argument("--speed-kmh", type=float, default=None); p.add_argument("--set", action="append")
    p = sub.add_parser("optimise", help="static geometric optimiser (no PX4)"); p.add_argument("--airframe", required=True); p.add_argument("--spec", default=None)
    p = sub.add_parser("export", help="PX4 .params file"); p.add_argument("--airframe", required=True); p.add_argument("--hitl", action="store_true"); p.add_argument("--out", default=None)
    p = sub.add_parser("paths", help="list variable paths"); p.add_argument("--airframe", required=True)
    sub.add_parser("scenarios", help="list bundled scenarios")
    p = sub.add_parser("migrate", help="convert a schema-1 airframe"); p.add_argument("src"); p.add_argument("dst")
    sub.add_parser("ui", help="interactive app (all further arguments go to it)")
    sub.add_parser("mcp", help="MCP server for AI assistants: `mcp` = stdio, `mcp --http [port]` = streamable HTTP")

    if argv and argv[0] == "mcp":
        from .mcp_server import main as mcp_main
        return mcp_main(argv[1:])
    if argv and argv[0] == "ui":
        return cmd_ui(None, argv[1:])
    if not argv:
        argv = ["ui"]
        return cmd_ui(None, [])
    a = ap.parse_args(argv)
    return {"run": cmd_run, "batch": cmd_batch, "study": cmd_study, "analyse": cmd_analyse, "optimise": cmd_optimise,
            "export": cmd_export, "paths": cmd_paths, "scenarios": cmd_scenarios, "migrate": cmd_migrate}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
