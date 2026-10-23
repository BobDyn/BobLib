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

## Physical record modifier map

The following are prefix replacements: keep the field suffix, including array
indices. `Fr` maps to `frontAxle` and `Rr` to `rearAxle`; both sides are required.
These paths are relative to the experiment instance.

| Old prefix | New prefix |
|---|---|
| `pVehicle.pFrAxleDW`, `pVehicle.pRrAxleDW` | `pVehicle.chassis.frontAxle.actuation`, `pVehicle.chassis.rearAxle.actuation` |
| `pVehicle.pFrDW`, `pVehicle.pRrDW` | `pVehicle.chassis.frontAxle.geometry`, `pVehicle.chassis.rearAxle.geometry` |
| `pVehicle.pFrRack`, `pVehicle.pRrRack` | `pVehicle.chassis.frontAxle.steering`, `pVehicle.chassis.rearAxle.steering` |
| `pVehicle.pFrAxleMass`, `pVehicle.pRrAxleMass` | `pVehicle.chassis.frontAxle.masses`, `pVehicle.chassis.rearAxle.masses` |
| `pVehicle.pFrPartialWheel`, `pVehicle.pRrPartialWheel` | `pVehicle.chassis.frontAxle.tire.wheel.geometry`, `pVehicle.chassis.rearAxle.tire.wheel.geometry` |
| `pVehicle.pFrTire1DOF_YParams`, `pVehicle.pRrTire1DOF_YParams` | `pVehicle.chassis.frontAxle.tire.wheel.rotation`, `pVehicle.chassis.rearAxle.tire.wheel.rotation` |
| `pVehicle.pFrTire1DOF_ZParams`, `pVehicle.pRrTire1DOF_ZParams` | `pVehicle.chassis.frontAxle.tire.wheel.vertical`, `pVehicle.chassis.rearAxle.tire.wheel.vertical` |
| `pVehicle.pFrTireModel`, `pVehicle.pRrTireModel` | `pVehicle.chassis.frontAxle.tire.force`, `pVehicle.chassis.rearAxle.tire.force` |
| `pVehicle.pFrStabar`, `pVehicle.pRrStabar` | `pVehicle.chassis.frontAxle.stabilizer`, `pVehicle.chassis.rearAxle.stabilizer` |
| `pVehicle.pBaseSprungMass` | `pVehicle.chassis.frame.baseMass` |
| `pVehicle.pDriverMass` | `pVehicle.chassis.frame.driverMass` |
| `pVehicle.pSprungMass` | `pVehicle.chassis.frame.mass` (derived; edit base/driver mass instead) |
| `pVehicle.pTorsionalStiff` | `pVehicle.chassis.frame.torsionalStiffness` |
| `pVehicle.pQSSInitialization` | `pInitialization` on `VehicleSim` |
| `pVehicle.pBattery`, `pVehicle.pVCU` | `pVehicle.battery`, `pVehicle.controller` |
| `pVehicle.pInverter`, `pVehicle.pMotor` | `pVehicle.inverter`, `pVehicle.motor` |
| `pVehicle.pAero` | `pVehicle.aero`, except ride-height references below |

For example, `pVehicle.pFrAxleDW.springFreeLength` becomes
`pVehicle.chassis.frontAxle.actuation.springFreeLength`, and
`pVehicle.pRrAxleDW.springTable[1,2]` becomes
`pVehicle.chassis.rearAxle.actuation.springTable[1,2]`. The intermediate
`actuation` member is necessary. Spring/damper tables also have active point
counts; preserve the count when editing a fixed-capacity exported table.

`pVehicle.pAero.FL_RideHeightRef` and `RL_RideHeightRef` move to
`pVehicle.chassis.frontAxle.rideHeightReference` and
`pVehicle.chassis.rearAxle.rideHeightReference`. These points belong to the
suspension measurements, rather than the aero force map.

Four-post bar-rate overrides need special care. `FourPostSim` explicitly binds
its axle stabilizers to the top-level `pFrStabar`/`pRrStabar` records and defaults
both rates to zero. Change `pFrStabar.barRate` and `pRrStabar.barRate` to enable
bars there; changing only the vehicle record's rate does not enable them.
Generated subclasses of `PartialDoubleWishboneFourPost` must retain those bindings:
`frAxleDW(pAxle(stabilizer=pFrStabar))` and
`rrAxleDW(pAxle(stabilizer=pRrStabar))` when they select stabilizer axles.

The driveline is not a uniform prefix substitution:

| Old field under `pVehicle.pDriveline` | New field or selection |
|---|---|
| `finalDriveRatio` | `pVehicle.transmission.gearRatio` (with `pVehicle.driveline.inputRatio=1`) |
| `diffInputRotorJ` | `pVehicle.driveline.inputInertia` |
| `diff_T_preload` | `pVehicle.driveline.differential.preloadTorque` |
| `diff_driveSideTorqueSign` | `pVehicle.driveline.differential.driveSideTorqueSign` |
| `diff_lockFractionAccel`, `diff_lockFractionDecel` | `pVehicle.driveline.differential.driveLockingFraction`, `coastLockingFraction` |
| `diff_kineticFrictionRatio` | `pVehicle.driveline.differential.slidingFrictionRatio` |
| `halfshaftLeftC`, `halfshaftRightC` | `pVehicle.driveline.halfshaft.stiffness` (shared by both shafts) |
| `halfshaftLeftD`, `halfshaftRightD` | `pVehicle.driveline.halfshaft.damping` (shared by both shafts) |
| `halfshaftLeftJEquivalent`, `halfshaftRightJEquivalent` | No behavior-preserving alias: `halfshaft.inertia` is now real rotating inertia, not just a damping calculation input |
| `diff_use_lsd` | Architecture selection `differentialArchitecture`; choose `Open` or `ClutchLSD` explicitly |
| `diff_T_capacity_max`, `diff_clutchEffectiveRadius`, `diff_w_transition`, `diff_c_viscous` | No direct alias in the new clutch law; recalibrate the selected differential |

Do not merge asymmetric left/right shaft calibration into the shared record
silently: use a custom driveline if independent calibration is required.
BobSim's generated records and runtime override dictionaries must both migrate.
Validate every requested override against the generated init XML or artifact
manifest and reject unknown keys; do not rely on the old runner's silent skip.

## Physical connections

Brakes is a top-level VI subsystem with front/rear assemblies. Rotor connections
and caliper-support frames have distinct roles: outboard calipers react through
uprights; inboard/differential-input brakes react through mounted housings.
Halfshafts transmit drive torque to wheel axes. Never connect one actuator to both
reaction paths or add a duplicate support torque. Mounts live in their owning
subsystems; keep attachment and mounted-housing frames distinct.

VehicleSim selects front-outboard/rear-inboard braking. Base models and other
wrappers can have different defaults: select the intended placement explicitly.
VehicleFMI retains its Modelica input boundary. Tools/export_fmu.py and
Tools/build_native.py preserve independent calibration before initialization;
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

## Maneuver exit status

`VehicleSim.terminationStatus` is included in the default result filter:

| Value | Meaning |
|---|---|
| 0 | Running, or the configured stop time was reached without a monitored outcome |
| 1 | Steady-state acceptance conditions held for the required duration |
| 2 | Steering lateral-gain loss limit reached |
| 3 | Settling timeout; not a converged point |
| 4 | Tire normal-load limit reached |
| 5 | Sideslip limit held for the configured duration |

Timeout and envelope exits use `terminate()` and retain the result file. A zero
process exit code alone does not establish convergence. Read the final status
and preserve limit points separately from accepted steady-state points. If limits
coincide, tire load takes priority over sideslip, timeout, linearity and settling.
Invalid calibration and solver failures remain errors. This output does not
change `VehicleFMI`'s externally driven command boundary.
