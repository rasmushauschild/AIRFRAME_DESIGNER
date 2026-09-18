px4_add_module(
 MODULE external__atlas_nose_lift
 MAIN atlas_nose_lift
 SRCS AtlasNoseLift.cpp
 MODULE_CONFIG params.yaml
 EXTERNAL
)

target_link_libraries(external__atlas_nose_lift PRIVATE px4_work_queue)
