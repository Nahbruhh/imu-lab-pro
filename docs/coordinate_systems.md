# Coordinate systems

Coordinate transformations must be explicit and tested. The Rafale model uses
a right-handed aircraft frame:

- +X: forward
- +Y: left
- +Z: up

Displayed roll is positive right-wing-down, pitch is positive nose-up, and yaw
is positive nose-right. Sensor mounting is represented by a proper rotation
from sensor coordinates into aircraft coordinates.

Structural projects must document global, specimen, sensor, and FEA coordinate
frames in their project configuration before comparing results.
