"""Parameter paths: address any numeric value of an airframe with a string, so optimisers, studies and AI
agents can vary "any set of parameters" without special cases.

Grammar
-------
  rotors[0].pos[2]            one element
  rotors[0,1].tilt_deg        several rotors at once (virtual attributes tilt_deg / cant_deg are supported)
  rotors[0:4].max_thrust      a slice
  rotors[*].tau               all
  rotors[M1].km               by name
  wings[0].incidence_deg      wing attributes; wings[0].aero.cd0 for coefficients
  legs[*].length
  mass.cg[0]                  the CG x coordinate
  hover_pitch_deg
  px4.MC_PITCHRATE_P          a PX4 parameter (stored in px4_overrides)
  design.cruise_speed_kmh     a design setting

set_path() writes; get_path() reads (returns a list when the path addresses several items).
"""
from __future__ import annotations

import copy
import re
from dataclasses import fields, is_dataclass
from typing import Any

_SEG = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)((?:\[[^\]]*\])*)")
_IDX = re.compile(r"\[([^\]]*)\]")


def _parse(path: str) -> list[tuple[str, list[str]]]:
    out = []
    for part in path.strip().split("."):
        m = _SEG.fullmatch(part)
        if not m:
            raise ValueError(f"bad path segment '{part}' in '{path}'")
        out.append((m.group(1), _IDX.findall(m.group(2))))
    return out


def _select(seq, spec: str) -> list[int]:
    spec = spec.strip()
    n = len(seq)
    if spec in ("*", ""):
        return list(range(n))
    if ":" in spec:
        a, b = spec.split(":", 1)
        return list(range(n))[slice(int(a) if a.strip() else None, int(b) if b.strip() else None)]
    idx = []
    for tok in spec.split(","):
        tok = tok.strip()
        if re.fullmatch(r"-?\d+", tok):
            i = int(tok)
            if not (-n <= i < n):
                raise IndexError(f"index {i} out of range (length {n})")
            idx.append(i % n)
        else:
            named = [i for i, x in enumerate(seq) if getattr(x, "name", None) == tok]
            if not named:
                raise KeyError(f"no item named '{tok}'")
            idx += named
    return idx


def _walk(obj: Any, segs: list[tuple[str, list[str]]]) -> list[tuple[Any, str | int]]:
    """Resolve a path to (container, key) pairs; a path may fan out over several items."""
    targets = [obj]
    for si, (name, idxs) in enumerate(segs):
        last = si == len(segs) - 1
        # px4.NAME and design.key: dict access
        nxt = []
        for t in targets:
            if isinstance(t, dict):
                if last and not idxs:
                    nxt.append((t, name)); continue
                child = t[name]
            elif name == "px4" and hasattr(t, "px4_overrides"):
                child = t.px4_overrides
            else:
                if last and not idxs:
                    nxt.append((t, name)); continue
                child = getattr(t, name)
            if idxs:
                for k, spec in enumerate(idxs):
                    sel = _select(child, spec)
                    if k == len(idxs) - 1 and last:
                        nxt += [(child, i) for i in sel]
                    elif k == len(idxs) - 1:
                        nxt += [child[i] for i in sel]
                    else:
                        child = [child[i] for i in sel]      # nested [..][..] on lists of lists
                        child = child[0] if len(child) == 1 else child
            else:
                nxt.append(child)
        targets = nxt
    return targets


def get_path(obj: Any, path: str):
    res = []
    for cont, key in _walk(obj, _parse(path)):
        res.append(cont[key] if isinstance(cont, (list, dict)) else getattr(cont, key))
    return res[0] if len(res) == 1 else res


def set_path(obj: Any, path: str, value) -> Any:
    """Set (in place) every item the path addresses. Returns obj."""
    for cont, key in _walk(obj, _parse(path)):
        if isinstance(cont, (list, dict)):
            cur = cont[key] if (isinstance(cont, list) or key in cont) else None
            cont[key] = _coerce(cur, value)
        else:
            if not hasattr(cont, key):
                raise AttributeError(f"'{type(cont).__name__}' has no attribute '{key}'")
            cur = getattr(cont, key, None)
            setattr(cont, key, _coerce(cur, value))
    return obj


def _coerce(current, value):
    if isinstance(current, bool):
        return bool(value)
    if isinstance(current, int) and not isinstance(value, bool):
        return int(round(value)) if isinstance(value, float) else value
    if isinstance(current, float):
        return float(value)
    return value


def apply_variables(airframe, values: dict[str, Any]):
    """Deep-copy the airframe and apply {path: value}; returns the new airframe."""
    new = copy.deepcopy(airframe)
    for p, v in values.items():
        set_path(new, p, v)
    if hasattr(new, "mass"):
        new.resolve_mass()
    return new


def list_paths(obj: Any, prefix: str = "", max_depth: int = 6) -> list[str]:
    """Every numeric leaf of a dataclass tree as a path string (for documentation and UI pickers)."""
    out: list[str] = []
    if max_depth < 0:
        return out
    if is_dataclass(obj):
        for f in fields(obj):
            v = getattr(obj, f.name)
            p = f"{prefix}.{f.name}" if prefix else f.name
            if f.name == "px4_overrides":
                out += [f"px4.{k}" for k in v]
            elif f.name == "design":
                out += [f"design.{k}" for k, x in v.items() if isinstance(x, (int, float)) and not isinstance(x, bool)]
            else:
                out += list_paths(v, p, max_depth - 1)
        for virt in ("tilt_deg", "cant_deg"):
            if hasattr(type(obj), virt):
                out.append(f"{prefix}.{virt}" if prefix else virt)
    elif isinstance(obj, list):
        if obj and isinstance(obj[0], (int, float)) and not isinstance(obj[0], bool):
            out += [f"{prefix}[{i}]" for i in range(len(obj))]
        else:
            for i, x in enumerate(obj):
                out += list_paths(x, f"{prefix}[{i}]", max_depth - 1)
    elif isinstance(obj, (int, float)) and not isinstance(obj, bool):
        out.append(prefix)
    return out
