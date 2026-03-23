# Changelog

## v1.1.0

New features from community feedback on the Chief Delphi thread. No API breaks, everything is additive.

- **New:** `ShotParameters` record and `ShotLUT` class for bundled RPM + hood angle + TOF lookup tables. Keeps all parameters in sync when interpolating, which matters for adjustable hoods. Based on a design suggested by Illinar (2702).
- **New:** `shooterAngleOffsetRad` config field for rear-facing shooters. Set to `Math.PI` and the solver rotates the drive heading so the back of the robot points at the hub. Requested by tcrvo on Chief Delphi.
- **New:** `generateVariableAngleShotLUT()` on ProjectileSimulator. Sweeps both RPM and angle at each distance, picks the lowest RPM. For teams with adjustable hoods.
- **New:** `magnusSign` parameter on ProjectileSimulator. Set to -1.0 for backspin shooters where Magnus pushes the ball down instead of up.
- **New:** `generateLUT(minDist, maxDist, step)` overload for custom distance ranges.
- **New:** `generateShotLUT()` convenience that returns a ShotLUT instead of a GeneratedLUT.
- **New:** `simulate(rpm, distance, angle)` and `findRPMForDistance(distance, angle)` overloads for specifying launch angle per-call.
- **New:** Static `rpmToExitVelocity()` and `exitVelocityToRPM()` helpers on ProjectileSimulator. No instance needed.
- **New:** `loadShotLUT()` and `getHoodAngle()` on ShotCalculator.
- **New:** GitHub Actions CI.
- **Docs:** Added sections for pre-tuned data, rear-facing shooters, adjustable hoods, unit conversions, backspin, custom LUT range, and calibration workflow.

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
