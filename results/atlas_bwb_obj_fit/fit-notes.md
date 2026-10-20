# ATLAS_BWB_CONCEPT_04 OBJ wing fit

Reference: ATLAS_JO_05B (1).obj, object primitive-up.006. Native OBJ units assumed metres. The mesh is registered to the prior root chord attitude and nose position: +23.081844 degrees pitch after mapping OBJ (-Z, X, -Y) into FRD. This removes its posed attitude without changing the model reference frame.

Sixteen mirrored UIUC wing segments replace eight. Sections come from actual mesh-edge intersections with spanwise planes. Leading-edge sweep, anhedral, root/tip chord and chord angle are fitted; slight tip twist approximates the curved surface. Existing NACA23018/23012 polar sections and NeuralFoil source remain. This fits the outline, not the exact OBJ thickness distribution.

Span: 3.50 -> 3.892 m. Surface planform area: 5.760 -> 6.402 square metres. Rear/cruise fan mounting heights follow the new wing surface; tripod attachment points follow the new nose/tips. Mass, inertia, rotor thrust ratings and PX4 tuning are retained.

Schema/geometry validation and static hover feasibility pass. No flight was run; existing tuning, inertia assumptions and ground clearance need revalidation for the changed shape.
