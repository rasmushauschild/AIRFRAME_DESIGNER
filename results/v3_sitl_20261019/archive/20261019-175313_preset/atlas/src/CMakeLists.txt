if(NOT CONFIG STREQUAL "px4_sitl_default")
 message(STATUS "ATLAS nose-lift in a board build (${CONFIG}): the module only runs when NLF_HW_OK = 1 (user opt-in)")
endif()
set(config_module_list_external atlas_nose_lift PARENT_SCOPE)
function(atlas_install_allocator_overlay)
 get_target_property(atlas_sources modules__control_allocator SOURCES)
 list(FILTER atlas_sources EXCLUDE REGEX "(^|/)ControlAllocator\.(cpp|hpp)$")
 set_property(TARGET modules__control_allocator PROPERTY SOURCES "${atlas_sources}")
 target_sources(modules__control_allocator PRIVATE "${EXTERNAL_MODULES_LOCATION}/allocator_overlay/ControlAllocator.cpp")
 target_include_directories(modules__control_allocator PRIVATE "${PX4_SOURCE_DIR}/src/modules/control_allocator")
endfunction()
cmake_language(DEFER DIRECTORY "${PX4_SOURCE_DIR}" CALL atlas_install_allocator_overlay)

file(SHA256 "${PX4_SOURCE_DIR}/src/modules/control_allocator/ControlAllocator.cpp" atlas_base_hash)
if(NOT atlas_base_hash STREQUAL "f000d3ccd8a7675c5eb3045a013ec9aaa08489f31341f56483e21f3ad5cc8a1b")
 message(FATAL_ERROR "ATLAS allocator overlay needs review against updated ControlAllocator.cpp")
endif()

file(SHA256 "${PX4_SOURCE_DIR}/src/modules/control_allocator/ControlAllocator.hpp" atlas_base_hash)
if(NOT atlas_base_hash STREQUAL "1bb3d045d11303d3dd3041754cb8755fcc7efa66c2dd607a1bed86fbac17d860")
 message(FATAL_ERROR "ATLAS allocator overlay needs review against updated ControlAllocator.hpp")
endif()

# In staged mode throttle belongs to the ground sequence. Preserve all attitude-stick takeover paths.
function(atlas_install_manual_overlay)
 file(READ "${PX4_SOURCE_DIR}/src/modules/manual_control/ManualControl.cpp" atlas_manual)
 set(atlas_throttle "|| (fabsf(_throttle_diff.update(_selector.setpoint().throttle, dt_s)) > velocity_threshold")
 string(FIND "${atlas_manual}" "${atlas_throttle}" atlas_found)
 if(atlas_found EQUAL -1)
  message(FATAL_ERROR "ATLAS staged throttle overlay needs review against ManualControl.cpp")
 endif()
 set(atlas_helper "
#include <uORB/topics/atlas_nose_lift_pilot.h>
#include <uORB/topics/vehicle_status.h>
static bool atlas_manual_attitude_active() {
 static uORB::Subscription sub{ORB_ID(atlas_nose_lift_pilot)};
 atlas_nose_lift_pilot_s state{}; sub.copy(&state);
 static uORB::Subscription status_sub{ORB_ID(vehicle_status)};
 vehicle_status_s status{}; status_sub.copy(&status);
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
 const bool simulation=true;
#else
 const bool simulation=status.hil_state==vehicle_status_s::HIL_STATE_ON && status.timestamp && hrt_absolute_time()-status.timestamp<2000000;
#endif
 return simulation && state.manual_attitude && hrt_absolute_time()-state.timestamp < 300000;
}
static bool atlas_pilot_throttle_active() {
 static uORB::Subscription sub{ORB_ID(atlas_nose_lift_pilot)};
 atlas_nose_lift_pilot_s state{}; sub.copy(&state);
 return state.active && hrt_absolute_time()-state.timestamp < 300000;
}
")
 string(REPLACE "#include \"ManualControl.hpp\"" "#include \"ManualControl.hpp\"${atlas_helper}" atlas_manual "${atlas_manual}")
 string(REPLACE "${atlas_throttle}" "|| (!atlas_pilot_throttle_active() && fabsf(_throttle_diff.update(_selector.setpoint().throttle, dt_s)) > velocity_threshold" atlas_manual "${atlas_manual}")
 set(atlas_assignment "_selector.setpoint().sticks_moving =")
 string(FIND "${atlas_manual}" "${atlas_assignment}" atlas_assignment_found)
 if(atlas_assignment_found EQUAL -1)
  message(FATAL_ERROR "ATLAS manual stick routing needs upstream review")
 endif()
 # Only the simulation prototype consumes attitude sticks itself. Physical mode switches remain available.
 string(REPLACE "${atlas_assignment}" "${atlas_assignment} !atlas_manual_attitude_active() &&" atlas_manual "${atlas_manual}")
 file(WRITE "${CMAKE_BINARY_DIR}/AtlasManualControl.cpp" "${atlas_manual}")
 get_target_property(atlas_manual_sources modules__manual_control SOURCES)
 list(FILTER atlas_manual_sources EXCLUDE REGEX "(^|/)ManualControl\\.cpp$")
 set_property(TARGET modules__manual_control PROPERTY SOURCES "${atlas_manual_sources}")
 target_sources(modules__manual_control PRIVATE "${CMAKE_BINARY_DIR}/AtlasManualControl.cpp")
 target_include_directories(modules__manual_control PRIVATE "${PX4_SOURCE_DIR}/src/modules/manual_control")
endfunction()
cmake_language(DEFER DIRECTORY "${PX4_SOURCE_DIR}" CALL atlas_install_manual_overlay)
