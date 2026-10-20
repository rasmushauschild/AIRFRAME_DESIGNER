# ATLAS_MVP_08B logged retry

Nose lift, takeoff and hover completed. The native PX4 velocity cruise aborted at the 65-degree tilt limit before reaching 50 km/h. Controlled nose landing was not reached.

The time series shows roll/yaw divergence precedes height loss. At north velocity 7.15 m/s (25.7 km/h), roll was 4.9 degrees and rising. At 8.49 m/s, roll was 23.9 degrees and peak fan utilization 98.2%. Individual actuator telemetry then shows asymmetric saturation (one rear motor at full command and another at zero). Abort at 11.73 m/s north velocity with tilt 65 degrees and downward speed 2.79 m/s.

This establishes the failure sequence, not the root cause. Investigate aerodynamic lateral moments, yaw/roll control allocation and available authority across speed before changing pitch or altitude control.

Files: airframe.json captures tested geometry; timeseries.json contains 50 Hz flight data and phase labels; telemetry.jsonl captures approximately 1 Hz PX4 telemetry including individual actuators; px4-log.json contains event logs; summary.json contains scenario results.
