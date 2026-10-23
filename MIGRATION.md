# Release-readiness migration

This migration changes public class paths and subsystem record ownership. Keep
existing calibrations under version control while updating callers. Dependencies
remain MSL 4.1.0 and VehicleInterfaces 2.0.2.

## Models and calibration

VehicleSim, VehicleFMI and FourPostSim remain under Experiments.Standards.
KnCSteerable and KnCLocked add single-axle rigs. The dynamic maneuver controller
remains; VehicleSimSS and GGV analysis are excluded.

Chassis.DoubleWishbone replaces the ChassisBase/Chassis_* family. Shared chassis
implementations live in Chassis.BaseClasses. Select front/rear axles independently
through their replaceable slots. Suspension implementations are position-independent;
mechanisms live in Suspension.Components and wheels in Suspension.Tires.Wheels.
Experiments.Standards.BaseClasses replaces Templates. Combination fixtures live in
BobLibTest, whose package paths now mirror production. See the standard Examples
and UsersGuide for current declarations and redeclaration examples.

Subsystems accept owner-named physical records such as pChassis, pAxle and pMotor,
passed down from the complete pVehicle record. Architecture selections, connector
switches and experiment settings remain separate. Old flat modifiers are not
compatibility aliases. Respect constrainedby contracts and keep implementation-only
modifiers on the selected implementation. Frame replacement is independent of axles.

## Old to new names used by BobSim

This table lists the names that BobSim on main writes today. It shows only names
that exist on main and on this release. BobSim's override writer silently skips
keys that the model does not have, so a caller with an old key gets no error.
Update the keys.

| Old name (main) | New name |
|---|---|
| `BobLib.Chassis.Chassis_DW` | `BobLib.Chassis.DoubleWishbone` |
| `Chassis_DW` modifier `redeclare record VehicleRecord` | `redeclare record ChassisRecord = VehicleRecord.ChassisRecord` |
| `Chassis_DW` modifier `pVehicle = pVehicle` | `pChassis = pVehicle.chassis` |
| `BobLib.Chassis.Suspension.FrAxleDW_Direct`, `RrAxleDW_Direct` | `BobLib.Chassis.Suspension.DoubleWishboneDirect` |
| `BobLib.Chassis.Suspension.FrAxleDW_BC`, `RrAxleDW_BC` | `BobLib.Chassis.Suspension.DoubleWishboneBellcrank` |
| `BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar`, `RrAxleDW_BC_Stabar` | `BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar` |
| `Experiments.Standards.Templates.Vehicle.BaseVehicleSim` | `Experiments.Standards.BaseClasses.PartialVehicleSim` |
| `Experiments.Standards.Templates.FourPost.BaseFourPostSim` | `Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost` |
| `vcu.targetAy`, `vcu.useMode`, `vcu.steerStart` | `driver.targetAy`, `driver.useMode`, `driver.steerStart` |
| `vcu.linearitySlopeSamplePeriod`, `vcu.handwheelRampRate`, `vcu.handwheelRampStopDuration` | `driver.` with the same parameter name |
| `vcu.enableNormalLoadSteerLimiter`, `vcu.tireNormalLoadMin` | `driver.` with the same parameter name |
| `vcu.steadyStateAyRampRate`, `vcu.steadyStateMaxHandwheel`, `vcu.steadyStateAyGain`, `vcu.steadyStateAyTi`, `vcu.steadyStateSteerTimeConstant` | `driver.` with the same parameter name |
| `vcu.frRampSteerHeight`, `vcu.stepDuration`, `vcu.steerAmp`, `vcu.steerFreq` | `driver.` with the same parameter name |
| `vcu.velGain`, `vcu.velTi` | `driver.velGain`, `driver.velTi` |

`initialVel` keeps its name and still sets the speed target through
`driver.targetVel`. The four-post modifiers `VehicleRecord`, `FrAxleModel`,
`RrAxleModel`, `pVehicle`, `pFrStabar` and `pRrStabar` keep their names on
`PartialDoubleWishboneFourPost`. `vcu.frRampSteerDuration` has no replacement.
On main it only set the default of `stepDuration`, so set `driver.stepDuration`.
The `Templates.Vehicle` and `Templates.FourPost` packages are removed, so put
generated templates in your own package.

## Physical connections

Brakes is a top-level VI subsystem with front/rear assemblies. Rotor connections
and caliper-support frames have distinct roles: outboard calipers react through
uprights; inboard/differential-input brakes react through mounted housings.
Halfshafts transmit drive torque to wheel axes. Never connect one actuator to both
reaction paths or add a duplicate support torque. Mounts live in their owning
subsystems; keep attachment and mounted-housing frames distinct.

VehicleSim selects front-outboard/rear-inboard braking. Base models and other
wrappers can have different defaults: select the intended placement explicitly.
VehicleFMI retains its Modelica input boundary. Tools/export_fmu.py preserves independent calibration before initialization;
derived parameters remain read-only. Rebuild old artifacts, and use the manifest
parameter contract described in Tools/README.md.

Wheel compliance is downstream of the caliper support. Dynamic models and static
rigs have distinct mass/damping settings; use the provided KnC/four-post models.
Do not zero dynamic vehicle mass or damping, or duplicate wheel spin inertia.

## Buses and validation

Declare signals in the owning subsystem Interfaces bus. BobLib ControlBus
aggregates these typed contracts. Do not duplicate ports to work around OMEdit's
inherited-member chooser: qualified connect equations with explicit routes remain
supported. Check connection types and equation balance with the compiler.

Run make ci and inspect failures and skips. Initialization baselines were migrated
with their fixtures; do not regenerate snapshots to silence mismatches. Static
route checks do not replace OMEdit review of inherited and conditional views.
See RELEASE_READINESS.md for current draft scope and remaining validation.
