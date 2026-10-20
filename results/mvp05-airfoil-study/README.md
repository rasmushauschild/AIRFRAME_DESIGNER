# MVP_05 airfoil and shape revision

Reference: ATLAS_JO_05B (1).obj, native units assumed metres. Existing span and planform retained. Centerbody upper/lower surfaces compared to primitive-up.006 and primitive-down.006. Fitting excludes motors and separate jetfoils. UIUC NACA 66-018 replaces NACA 23018 inboard; profiles blend into UIUC S1210 outboard. The 60 mm thickness constraint was withdrawn. Original model is in before.json.

Centerbody position and chord angles adjusted to fit both arcs. Compared with the original centerbody, thickness at 1% chord decreases from 5.38% to 3.52% of chord. Outer S1210 decreases this measure from NACA23012's 3.56% to 3.46%. This is a leading-edge thickness proxy, not a fitted nose-radius measurement.

At 120 kg and 50 km/h: static sweep predicts vertical wing force of 100.08% of weight at 24.25 degrees body pitch, 428 N wing drag, and -239 Nm pitching moment. Peak predicted support is only 104.02% at 27.5 degrees. Geometric stall warning is active at those conditions. These numbers do not establish safe stall margin, efficient cruise, or trim. The app's existing cruise solver returns a motor-supported negative-pitch condition rather than the required wing-borne condition. The requested 100% lift in balanced cruise is therefore NOT demonstrated.

Calculations use NeuralFoil section polars with the existing strip/downwash model, not CFD or wind-tunnel measurements of this swept low-aspect-ratio aircraft. Dynamic flight was not run. The polar model's geometric stall flag and per-segment downwash approximation limit interpretation near maximum lift. No thrust axes, motor ratings, CG, mass or inertia were changed. Further trim/control allocation and 3D aerodynamic validation are needed.

Sources: https://m-selig.ae.illinois.edu/ads/coord/naca66-018.dat and https://m-selig.ae.illinois.edu/ads/coord/s1210.dat .
