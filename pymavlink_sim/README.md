# ArduPilot Non-GPS SITL Simulation

This folder contains the Mission Planner / ArduCopter SITL setup for testing indoor navigation using simulated visual odometry instead of GPS. A Python MAVLink bridge supplies synthetic vision-position measurements to EKF3; it is **not** a camera-based visual odometry implementation.

## Requirements

- Mission Planner with ArduCopter SITL
- Python 3 and `pymavlink` (`python -m pip install pymavlink`)
- A MAVLink TCP endpoint accessible to `vision_bridge.py` (check the host and port in the script)
- The parameter and mission files below

## Files

```text
pymavlink_sim/
├── README.md
├── vision_bridge.py
├── params/
│   ├── baseline_gps_original.param
│   └── baseline_nongps_v0.param
└── missions/
    ├── phase1_hover.waypoints
    └── phase2_square.waypoints
```

- `baseline_gps_original.param`: GPS-enabled SITL backup (previously `GPS_SITL_Backup.param`; verify it represents the original baseline).
- `baseline_nongps_v0.param`: non-GPS parameter export; **recreate, export and validate this file before submitting** if it was not saved during the original run.
- `phase1_hover.waypoints`: take off to approximately 1 m, hold for 30 seconds, then land.
- `phase2_square.waypoints`: fly an approximately 1.5 m square at low speed, then land (not RTL).
- `vision_bridge.py`: streams simulated position measurements as MAVLink `VISION_POSITION_ESTIMATE` messages, targeting approximately 15 Hz.

## Reproduce the simulation

1. Start ArduCopter SITL in Mission Planner and connect to the vehicle.
2. In **CONFIG → Full Parameter List**, load `params/baseline_nongps_v0.param`, write the parameters, restart SITL, and reconnect. Verify the parameters actually applied.
3. Confirm GPS is disabled, visual odometry input is enabled, EKF3 is configured for external navigation, and the intended low-speed and altitude limits are set. Parameter names and accepted values depend on the ArduCopter firmware version.
4. Check `vision_bridge.py` for its MAVLink host/port, SITL ground-truth source, coordinate conventions and required arguments. Configure a **separate** MAVLink connection so the bridge does not conflict with Mission Planner.
5. Run the bridge from a terminal (if the script takes no required arguments):

   ```bash
   python vision_bridge.py
   ```

6. With the vehicle still on the ground, verify that the script is sending estimates and Mission Planner reports EKF3 using external navigation. Do not proceed if external navigation is unhealthy.
7. In the **PLAN** tab, load `missions/phase1_hover.waypoints`, write the mission, and run it in SITL. Observe position drift during the 30-second hold and verify landing.
8. Load `missions/phase2_square.waypoints`, write the mission, and run it at low speed. Confirm that the vehicle lands rather than initiating RTL.
9. Save the flight logs and record the ArduCopter version, parameter export, observations, and a screen recording for review.

## Validation and limitations

- Check the configured altitude fence in SITL; a fence setting alone does not guarantee a physical 1.5 m ceiling.
- Confirm the vision bridge uses **live SITL ground-truth position**, not a prerecorded trajectory, and that coordinate frames and timestamps are consistent.
- Record any EKF failsafes, loss of external navigation, unexpected drift, or landing issues.
- Delays, noise and vision dropouts are additional tests; do not treat them as completed unless tested and documented.
- This is a software-in-the-loop validation. Hardware flight with AprilMAV or another real visual-odometry source requires separate integration and testing.

## Submission note

The non-GPS parameter export was not retained from the earlier session. If `baseline_nongps_v0.param` is newly recreated, label it as a **reconstructed configuration** until it has been tested again. Do not present it as an original export from that session.
