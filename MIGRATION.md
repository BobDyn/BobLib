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
