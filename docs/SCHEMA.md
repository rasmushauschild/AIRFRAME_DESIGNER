# Airframe JSON schema (version 2)

Everything lives in the **structural frame**: FRD (x forward, y right, z down), metres, origin at the airframe's
reference point (any convenient datum). The centre of gravity is a property of the `mass` segment, so moving the
CG never means re-entering positions. Angles are degrees in JSON.

```jsonc
{
  "schema": 2,
  "name": "ATLAS_08",
  "mass": {
    "mass": 13.0,                 // kg, total
    "cg": [0.0, 0.0, 0.0],        // centre of gravity, structural frame
    "inertia": [0.075, 0.128, 0.185],      // Ixx Iyy Izz about the CG, body axes
    "inertia_products": [0, 0, 0],         // Ixy Ixz Iyz
    "items": [ {"name": "battery", "mass": 2.0, "pos": [0.1, 0, 0.02], "inertia": [0, 0, 0]} ],
    "from_items": false           // true: mass, cg and inertia are computed from items
  },
  "body": {
    "size": [0.14, 0.12, 0.06],   // drawn box, m
    "drag_quadratic": [0.1, 0.1, 0.2],     // N per (m/s)^2 along each body axis
    "drag_angular": [0.005, 0.005, 0.005], // Nm per (rad/s)^2
    "drag_center": [0, 0, 0]      // where the body drag acts
  },
  "rotors": [ {
    "name": "M1", "enabled": true,
    "pos": [-0.3, -0.37, 0.14],   // structural frame
    "axis": [0.26, 0.0, -0.97],   // unit thrust direction (up = [0,0,-1]); the UI edits it as tilt/cant
    "km": 0.01,                   // CA_ROTORn_KM: reaction torque per unit thrust, sign = spin (>0 CCW from above)
    "max_thrust": 36,             // N at full command
    "tau": 0.12,                  // spool time constant, s
    "diameter": 0.08,             // fan/prop diameter, m
    "thrust_exponent": 2.0,       // thrust = max * omega^n
    "kind": "ducted",             // "prop" | "ducted"
    "ram_drag": true,             // momentum drag of the inlet flow (ducted fans)
    "duct_axis": [1, 0, 0],       // jetfoil: physical fan axis; the jet is bent to "axis" (null = fan along the jet)
    "turn_loss": 0.1              // thrust lost at 90 degrees of jet deflection
  } ],
  "wings": [ {
    "name": "main", "enabled": true,
    "pos": [0.42, 0.0, 0.06],     // ROOT LEADING EDGE on the centreline
    "root_y": 0.0,                // optional mirrored lateral root offset; lets adjacent segments join without overlap
    "span": 1.075,                // tip to tip (symmetric) or panel length (single panel)
    "root_chord": 0.93, "tip_chord": 0.0,
    "sweep_deg": 60,              // leading-edge sweep, back positive
    "dihedral_deg": 0,            // tips up positive; 90 with symmetric=false is a vertical fin
    "incidence_deg": 10,          // section incidence at the root (each section turned about the span line), LE up
    "pitch_deg": 0,               // whole-wing pitch: rigid rotation about the aircraft pitch axis through pos, nose-up positive
    "twist_deg": 0,               // extra incidence at the tip (washout negative)
    "symmetric": true,            // mirrored left/right halves
    "panels": 6,                  // strips per half for the strip-theory model
    "aspect_ratio": null,         // override the geometric AR
    "aero": {
      "model": "polhamus",        // "linear" (slope + induced drag) | "polhamus" (sharp-edged delta with vortex lift)
                                  // | "polar" (airfoil section data: CL/CD/CM vs alpha and Re from XFOIL or NeuralFoil)
      "airfoil_root": "naca23006", // polar model: NACA 4/5-digit, a file airfoils/<name>.dat, or a UIUC database name
      "airfoil_tip": "naca23006",  // polar model: tip section (blank = root); sections blend linearly along the span
      "polar_source": "auto",      // "auto" (XFOIL if installed, else NeuralFoil) | "xfoil" | "neuralfoil"
      "ncrit": 9.0,                // transition criterion of the polar (9 clean, 5 rough/turbulent)
      "cl_alpha": 6.2832,         // 2-D lift slope, 1/rad (Helmbold gives the 3-D slope from the AR)
      "cl0": 0.0, "cd0": 0.02, "oswald": 0.85,
      "stall_deg": 30, "stall_blend_deg": 15, "cd_flat": 1.2,
      "cm0": 0.0,                 // section pitching moment about the quarter chord
      "vortex_lift": true
    }
  } ],
  "legs": [ {
    "name": "FR", "enabled": true,
    "attach": [0.2, 0.2, 0.0],    // where the leg meets the structure
    "length": 0.28,
    "tilt_deg": 10,               // leg leans forward (+) / back (-)
    "cant_deg": 0,                // leg leans outward (+) / inward (-)
    "foot_radius": 0.0,           // contact starts this far above the foot point
    "stiffness": 3000, "damping": 150, "friction": 0.8
  } ],
  "hover_pitch_deg": 15,          // nose-up pitch PX4 treats as level (geometry exported in that frame, SENS_BOARD_Y_OFF set)
  "landed_pitch_deg": -10,        // attitude when standing on the legs (initial state of a simulation)
  "px4_overrides": {"MPC_THR_HOVER": 0.6},   // PX4 parameters set by hand, exported and seeded with the geometry
  "design": {"cruise_speed_kmh": 50},        // analysis / optimiser settings
  "notes": ""
}
```

Schema 1 files (the AIRFRAME_SIMULATOR format) load transparently: `Airframe.from_dict()` migrates them (mass number
→ `mass`, `prop_diameter` → `diameter`, generated feet → four `legs`, the area/span delta wing → a `polhamus`
wing positioned by its root leading edge, CG = origin).

## Parameter paths

Any numeric leaf can be addressed by a string, used by `--set`, studies and the API:

| path | meaning |
|---|---|
| `rotors[0].pos[2]` | one element |
| `rotors[0,1].tilt_deg`, `rotors[0:8].cant_deg`, `rotors[*].tau` | several rotors (virtual `tilt_deg` / `cant_deg` write the axis) |
| `rotors[M3].km` | by name |
| `wings[0].incidence_deg`, `wings[0].aero.cd0`, `wings[fin].sweep_deg` | wings |
| `legs[*].length`, `legs[FL].tilt_deg` | legs |
| `mass.cg[0]`, `mass.mass`, `mass.inertia[1]` | mass segment |
| `hover_pitch_deg`, `landed_pitch_deg` | attitudes |
| `px4.MC_PITCHRATE_P` | a PX4 parameter (goes into `px4_overrides`) |
| `design.cruise_speed_kmh` | design settings |

`airframe-designer paths --airframe X` prints every path with its current value.

For joined wing segments, `span` covers the segment itself (both halves if symmetric), excluding `root_y`.
`root_y` offsets the two roots by ±y; it adds no aerodynamic area. Adjacent segments should meet at their
leading and trailing edges. A shared whole-planform aspect-ratio override is only a strip-theory approximation
to aerodynamic interaction between segments, not a validated blended-body flow solution. Optional
`design.visual.wing_colors` supplies CSS colors in wing order for the 3-D solid surfaces.

## `cad` (optional): STEP bodies as masses

```jsonc
"cad": {
  "file": "airframes/cad/atlas_frame.step",   // copied there by the Geometry tab's "Import STEP…"
  "rotation_deg": [-90, 0, 0], // rotation of the CAD model about the aircraft x, then y, then z axes (a Y-up export needs x = -90)
  "origin": [0, 0, 0],         // FRD position of the CAD origin, m
  "scale": 1.0,                // extra factor on the geometry (STEP units are converted to metres automatically)
  "visible": true,             // draw the bodies in the 3D view
  "bodies": [
    { "id": "0:Battery", "name": "Battery",
      "mass": 5.0,             // kg, typed by the user (0 = the body is ignored)
      "offset": [0.02, 0, 0],  // where the user dragged it, FRD m, relative to the CAD position
      "removed": false,        // dropped from the list (kept so it can be restored)
      "volume": 0.00226,       // measured by OpenCascade, CAD frame, m^3
      "centroid": [0.3, 0, 0.2],
      "inertia_unit": [9.6e-6, 9.6e-6, 4.1e-6, 0, 0, 0]   // Ixx Iyy Izz Ixy Ixz Iyz about the centroid for density 1 kg/m^3
    }
  ]
}
```
With `mass.from_items` true the aircraft mass, CG and inertia are computed from these bodies (each body's mass at its
centroid + offset, with the solid's inertia scaled to its mass) plus any hand-made `mass.items`. Every body field is
addressable as a parameter path: `cad.bodies[Battery].mass`, `cad.bodies[0].offset[0]`, `cad.rotation_deg[0]`, `cad.origin[2]`. The meshes for
the 3D view are cached beside the STEP file as `<file>.bodies.json` (regenerated when the file changes; not committed).
`MassItem` gained `inertia_products` (Ixy Ixz Iyz of the item's own inertia) for the same reason.
