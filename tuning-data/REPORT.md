# Tuning report

From TunerTeleOp sessions: 20280507-230429 (6640 rows)

Written into `TeamCode/.../Constants/TunedConstants.java`. Flywheel gains are never changed, the turret model comes from the Turret Autotuner.

- **Loop time** (logging reads removed): median 29.4 ms, p95 41.8 ms, p99 48.7 ms, worst 76.9 ms over 6639 loops.
- **Flywheel voltage filter:** period 0.020 s, tau 0.040 s. Tracking error 0.1488 V vs 0.1746 V with the old one. Sag seen 3.21 V.
- **Turret voltage filter:** period 0.020 s, tau 0.040 s.
- **flywheel velocity filter:** 0.25-13.6 Hz, memory 0.80, stale 0.112 s (hub updates every 0.045 s). Next-reading error 20.9 ticks/s (was 20.9).
- **turret velocity filter:** 0.50-32.0 Hz, memory 0.90, stale 0.085 s (hub updates every 0.034 s). Next-reading error 2031.3 ticks/s (was 2031.3).
- **Flywheel vibration** (shooter on, settled, not feeding): 13.2 ticks/s RMS, about 1.7 Hz. Shot gate bands 26 / 52, dwell 0.060 s.
- TURRET_VIBRATION_RMS kept: not enough turret holding time.
- Drivetrain model kept: no finished Drivetrain Autotuner run pulled yet.
- **Turret** from Turret Autotuner 20280512-230114: 149.7 deg/s per V, friction 1.18 / 1.18 V, score 11.244079791976956 -> 4.032145725333396.
