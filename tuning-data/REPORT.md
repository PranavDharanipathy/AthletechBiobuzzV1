# Tuning report

From TunerTeleOp sessions:

Written into `TeamCode/.../Constants/TunedConstants.java`. Flywheel gains are never changed, the turret model comes from the Turret Autotuner.

- **Turret model** from Turret Autotuner 20280507-081142-driving (driving): 166.6 deg/s per V, lag 46 ms, dead time 36 ms, friction 1.01 / 1.16 V, bandwidth 3.2 rad/s.
  - driving_tracking_error_deg: 25.407
  - driving_hold_vibration_deg_s: 70.74
