# Architecture

IMU LAB Pro separates measurement acquisition, calibration, analysis,
simulation, visualization, storage, and UI concerns. Project-specific inputs
and solver artifacts live under `projects/`; reusable implementation lives
under `src/imu_lab_pro/`.

```text
physical sensor -> acquisition -> calibration -> analysis
                                             -> model comparison
                                             -> state reconstruction
                                             -> visualization/reporting
```

The current application remains a prototype while these boundaries are
extracted incrementally. New modules should be introduced only when they own
real behavior and have tests.
