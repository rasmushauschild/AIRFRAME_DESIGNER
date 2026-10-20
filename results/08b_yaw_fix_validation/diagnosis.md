# Yaw correction validation

Geometry unchanged. Increased MC_YAWRATE_K from 0.6 to 3.0 and refreshed CA_ROTOR0_CT through CA_ROTOR8_CT proportional to effective thrust after jetfoil turning losses. PX4 acknowledged the parameter push.

Offline local linearization predicted loss of lateral-directional stability with the prior yaw gain at cruise speed. The allocation torque sign was correct. Prior CT values were stale relative to current jetfoil axes. These two changes were tested together, so their individual contributions are not isolated.

Visible SITL passed takeoff, hover, acceleration, 50 km/h target cruise, deceleration and hover. Cruise mean speed 13.5 m/s (48.6 km/h), yaw drift -0.812 degrees, maximum roll 1.735 degrees, maximum utilization 0.6008. Prior runs diverged before reaching cruise.

Landing contacted at 0.40 m/s and entered rear-leg support/nose lowering. It subsequently timed out while still armed after reaching the resting pitch. This is a separate unresolved landing completion issue. The run as a whole did not pass. No claim for 100 km/h or real-aircraft validation.
