# Structural Vibration Lab

Structural Vibration Lab is the experimental validation subproject of IMU LAB
Pro. It defines repeatable structural-dynamics experiments that connect
physical measurements, analytical theory, and numerical simulation.

The lab is not a separate software implementation. Reusable acquisition,
calibration, analysis, storage, and visualization code belongs under
`src/imu_lab_pro/`.

## Experiments

| ID | Experiment | Status |
|---|---|---|
| 001 | [Cantilever beam vibration](experiments/001_cantilever_beam/) | Planning and measurement validation |

## Evidence policy

- Unknown engineering inputs remain `null` until measured or sourced.
- Raw captures stay under `data/raw/` and are not committed.
- Only reviewed, sanitized examples may be published with an experiment.
- Measured, calculated, simulated, and reconstructed quantities must be
  labeled separately.
- Hardware-dependent claims require retained data and test metadata.

Shared conventions are maintained in [`shared/`](shared/). Experiment-specific
deviations must be justified in that experiment's documentation.
