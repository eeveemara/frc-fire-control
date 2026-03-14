# Changelog

## v1.0.1

Bug fixes for the SOTM solver. No API changes, drop-in replacement for v1.0.0.

- **Fixed:** Newton derivative now includes `e^(-ct)` drag factor (chain rule on `dragCompensatedTOF`). The solver already converged via warm start, but Newton steps were ~20% off, occasionally needing an extra iteration. Hat tip to Illinar (2702) on Chief Delphi for catching this.
- **Fixed:** Angular velocity feedforward sign was inverted. `driveAngularVelocityRadPerSec()` now returns the correct direction so heading feedforward helps the PID instead of fighting it.
- **Fixed:** Comment on heading distance scaling said "Closer = tighter" when the code correctly implements farther = tighter (a 5-degree error at 5m misses by 44cm, at 1m only 8.7cm).
- **Optimization:** Compute `e^(-ct)` once per Newton iteration instead of twice (once for drift, once for derivative). Suggested by Illinar (2702).

## v1.0.0

Initial release. Three standalone fire control classes for FRC 2026 REBUILT.

- **ShotCalculator** - Newton-method SOTM solver with drag compensation, warm start, second-order pose prediction, speed/distance-scaled heading tolerance, tilt suppression, and confidence scoring
- **ProjectileSimulator** - RK4 projectile sim with drag and Magnus lift, generates 91-point shooter LUTs
- **FuelPhysicsSim** - Full-field ball physics simulation (2026 REBUILT field geometry)
