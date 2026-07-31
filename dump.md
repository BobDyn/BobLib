# BobLib — Full Repository Dump

Snapshot of `C:\Users\raorj\code\lhr\BobLib` at commit `6e28ebb` ("Release BobLib 0.1.1"),
branch `main`. Everything below was read out of the source, not inferred from docs.

> **Note:** the pre-existing `knowledge.md` in this repo is stale. It describes a top-level
> layout of `Resources / Standards / Vehicle / Utilities / Tests` with entry points like
> `Vehicle.VehicleDW_RWD_Lock` and `Resources.VehicleDefn.OrionRecord`. None of that exists
> anymore. The current layout is documented here.

---

## 1. What this is

BobLib is a **standalone Modelica library for detailed FSAE-style electric vehicle dynamics**.
It is the low-level modelling layer of a product family called **BobDyn**:

| Layer | Role |
|---|---|
| **BobLib** (this repo) | Modelica physics: chassis, suspension, tires, powertrain, aero, controllers, plus runnable standard experiments and regression fixtures. |
| **BobSim** (separate) | Workflow layer: study execution, signal extraction, metrics, plots, reports, envelope maps, sensitivity sweeps. |
| **BobDocs** (separate) | Web mirror of documentation. In-package `BobLib.UsersGuide` is the authority. |

Design thesis: **every public subsystem is a VehicleInterfaces contract; all BobLib-specific
physics lives one level deeper.** So a `Chassis`, `Motor`, `Driveline`, etc. can be swapped for
anyone else's VehicleInterfaces-compliant model without touching the vehicle assembly.

Licence GPL-3.0. Version `0.1.1`, dated `2026-07-05`.

### Stack

- Modelica Standard Library **4.1.0**
- VehicleInterfaces **2.0.2**
- OpenModelica **v1.26.3** (CI container `openmodelica/openmodelica:v1.26.3-ompython`)
- Python 3.11 + `pytest` + `ruff` for the test harness

### Inventory

```
273  .mo files under BobLib/
 45  .mo files under Tests/ (the sibling BobLibTest package)
 89  model
 34  partial model
 46  record
 19  function  (+1 pure function)
 85  package   (+6 partial package)
  1  expandable connector   (BobLib.Atmospheres.Interfaces.AtmosphereBus)
```

Only **one** BobLib-owned connector exists (the atmosphere bus). Everything else rides on
VehicleInterfaces' `ControlBus` and MSL MultiBody / Rotational / Electrical connectors — that
is deliberate and is the single most important architectural rule in `AGENTS.md`.

### Entry points

| Model | What it does |
|---|---|
| `BobLib.Experiments.Standards.VehicleSim` | Full-vehicle manoeuvre benchmark. Default: open-loop ramp steer at 15 m/s, 10 s. |
| `BobLib.Experiments.Standards.FourPostSim` | Four-post K&C rig, 118 s scripted heave → roll sweep. |
| `BobLib.Experiments.Standards.VehicleFMI` | Driver-input FMI / DIL / SIL vehicle boundary. |

---

## 2. Repository layout

```
BobLib/                      the Modelica package
├── UsersGuide/              7 HTML tutorial pages (in-package, versioned, authoritative)
├── Experiments/Standards/   runnable entry points + template families
├── Records/                 all parameter data
│   ├── VehicleDefn/         9 complete vehicle definitions (the "cars")
│   ├── VehicleRecord/       per-subsystem record types (mirrors public domains)
│   └── StandardRecord/      test/eval output records
├── Icons/                   26 reusable icon classes
├── Aero/                    CFDAeroMap + Interfaces.Base + Mounts + Bilinear2D
├── Atmospheres/             ConstantAtmosphere + AtmosphereBus
├── Chassis/                 Body, Suspension (incl. Tires), Brakes, Internal, Chassis_*
├── ElectricDrives/          Motor + Internal.PowerLimitedMotor
├── Engines/                 SimpleICEngine + Internal.SimpleICEngineCore
├── Transmissions/           FixedRatioTransmission + Internal.FixedRatioGear
├── PowerElectronics/        InverterDC
├── Controllers/             VCU, StandardVCU, Internal.VCUCore
├── DriverEnvironments/      AutomaticDriveByWire, EVDriveByWire, Internal.Driver
├── Drivelines/              RearFinalDriveDifferential + Internal.Differential1D / LockedDifferential1D
├── EnergyStorage/           BatteryPack + Internal.TheveninBatteryPack
└── Utilities/               Math (Vector/Tensor), Mechanics (Functions/MultiBody), FMI

Tests/
├── BobLibTest/              sibling Modelica test library (component + regression models)
├── modelica_linter.py       formatter/linter (120-col, layout rules)
├── modelica_translation_checks.py
├── modelica_initialization_checks.py
├── test_*.py                6 pytest modules
└── *_baseline.csv           initialization / physics / runtime baselines
```

The layering rule (from `AGENTS.md`, and it is actually followed):

1. **First level of a public domain package** = the VehicleInterfaces-facing adapter. These are
   the classes you redeclare into a vehicle.
2. **`Internal/`, `Templates/`, `Mounts/`, `Actuators/`** = BobLib implementation detail, one
   level deeper. Never a second public access path to the same physics.
3. **Tires live under `Chassis.Suspension`** because axles own wheel centres and load paths.
4. **Contact/ground closure is NOT in `Tires`.** Axles expose *raw* contact-patch frames; the
   chassis closes them to ground via `Utilities.Mechanics.MultiBody.ContactMechanics`.
5. **Tests live in `Tests/BobLibTest`**, never in production packages.

---

## 3. Coordinate and sign conventions

This is the part you need to get right before touching any hardpoint.

| Item | Convention |
|---|---|
| Frame | **Z-up, X-forward, Y-left** (ISO-ish). `world(n = {0,0,-1})`. |
| Origin | Front-axle wheel-centre plane. `pFrDW.wheelCenter[1] = 0`, rear is at `x = -1.5494`. |
| Lateral | **Left = +Y.** All records store the **left** side only. |
| Mirroring | `Utilities.Math.Vector.mirrorXZ` flips Y. `Tensor.mirrorXZ` mirrors an inertia tensor. `AxleDWBase` builds the right side by mirroring every left record automatically. |
| Aero | `force = {-drag, 0, -downforce}` in the body frame (so positive `downforce` pushes down). |
| Tire | MF5.2 is evaluated in its native (SAE-ish) frame, then `Eval` flips `Fy`, `My`, `Mz` at the boundary — see the "Z-up transform (apply at boundary)" block. |
| Toe/camber | `staticAlpha` / `staticGamma` are documented **in degrees** but typed `SI.Angle`. See §9 finding F4. |

---

## 4. What is actually modelled

### 4.1 Chassis and suspension (the detailed part)

**Topology per axle** (`Chassis.Suspension.AxleDWBase`, partial):

```
axleFrame (from space frame)
 ├─ toLeftUpper_i  ──► leftWishboneUprightLoop.upperFrame_i
 ├─ toLeftLower_i  ──► leftWishboneUprightLoop.lowerFrame_i
 ├─ toRack ──► rackAndPinion ──► leftTieRod / rightTieRod ──► steeringFrame
 └─ (mirrored right side)
                     leftWishboneUprightLoop.steeringFrame
                       └─ toLeftWheelCenter ──► leftTire.chassisFrame ──► leftCP (Frame_b)
leftTorque (Flange_b) ──► leftTire.hubFlange
```

`WishboneUprightLoop` is a real closed kinematic loop, not a lookup table:

- Upper wishbone + upright = `Joints.Assemblies.JointUSR` (Universal–Spherical–Revolute
  aggregate — MSL's analytic loop solver, so no iteration for that loop).
- Lower wishbone = a plain `Revolute` about the `lowerFore_i → lowerAft_i` axis, with
  `stateSelect = StateSelect.always` — **this is the chosen state for the corner.**
- Steering axis = `Revolute` about the normalised `upper_o → lower_o` (kingpin) vector.
- Tie rod = `Rod` (`UniversalSpherical`, `kinematicConstraint = true`).
- Four `FixedShape` cylinders drive animation of the four wishbone legs (lengths recomputed
  live from frame positions, so the arms visually track motion).

Masses: `Body` elements for UCA, LCA and the unsprung corner, with `r_CM` expressed relative to
the hardpoint they hang off (`rCM - upper_o`, `rCM - lower_o`, `rCM - wheelCenter`).

**Three axle architectures** (front and rear variants of each, 6 concrete models):

| Model | Actuation |
|---|---|
| `FrAxleDW_Direct` / `RrAxleDW_Direct` | Coilover mounted directly between a wishbone and the chassis. |
| `FrAxleDW_BC` / `RrAxleDW_BC` | Pushrod/pullrod → `Bellcrank2` (2 pickups) → shock. |
| `FrAxleDW_BC_Stabar` / `RrAxleDW_BC_Stabar` | Pushrod/pullrod → `Bellcrank3` (3 pickups) → shock **and** droplink → anti-roll bar. |

`Bellcrank3` is a `Revolute` about `pivotAxis` plus a chain of `FixedTranslation`s to the three
pickups, with four `FixedShape` cylinders for the crank plate outline.

`ShockLinkage` = `LineForceWithMass` between two frames, with `TabularSpring` (force-vs-deflection
`CombiTable1Ds`, `LinearSegments`, `LastTwoPoints` extrapolation) and `TabularDamper`
(force-vs-velocity) across it. Both use a smooth `sqrt(x² + eps²)` signum so there is no event at
zero deflection/velocity.

`Stabar` = rigid arms (`FixedTranslation`) + one `Revolute` about Y with a rotational `Spring` of
rate `barRate` between axis and support, connected to the bellcranks through
`SphericalSpherical` droplinks.

`RackAndPinion` = `Prismatic` along Y + `IdealGearR2T` with `ratio = 2π / cFactor`
(cFactor = m of rack travel per pinion revolution), plus `Mounting1D` for the reaction path.

**Chassis body** (`Chassis.Body`):

- `FrameBase` (partial): front/rear axle frames, sprung `Body` at `pSprung.rCM`, `cgFrame`.
- `FrameRigid`: rigid link between the two axle frames.
- `FrameCompX`: a **torsional DOF about X** between the two axle frames, with `torsionalStiff`
  spring and critically-damped damper (`2·√(k·Ixx)`). This is the chassis-torsion model.
  `StateSelect.always` on that revolute.

**Contact / ground** (`Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics`):

```
pen   = smooth(1, 0.5*(sqrt(r_rel_z² + eps²) - r_rel_z))   // smooth max(-Δz, 0)
f_raw = c*pen + d*der(pen)
f_z   = smooth(1, 0.5*(sqrt(f_raw² + forceEps²) + f_raw))  // smooth positive part
```

Fully event-free unilateral contact. `c = 100000 N/m`, `d = 750 N·s/m`, applied purely in world Z.
Chassis-level, four instances, each between a `Parts.Fixed` at the design contact-patch position
and the axle's raw CP frame.

### 4.2 Tires

`BaseTire` is the assembly; `MF52Tire` extends it and fills in the five force/torque expressions.

- **Wheel physics** (replaceable, `TirePhysics.Templates.PartialWheel` base):
  - `Wheel0DOF` — rigid, used for the four-post rig.
  - `Wheel1DOF_Y` — spinning wheel with inertia `wheelJ`, **rigid vertical** (a `Rod` of length
    `R0` locks `prismatic_z`). This is what the standard vehicle uses.
  - `Wheel1DOF_Z` — vertical tire spring/damper (`wheelC`, `wheelD`), no spin DOF.
  - `Wheel2DOF_YZ` — both.
  - Common: `FixedRotation` for static camber/toe, `Revolute` hub axis, `Prismatic` along −Z,
    `VoluminousWheel` visualiser, and a `Product` block generating rolling torque
    `longitudinalTorqueSign · cpFrame.f[1] · loadedRadius`.
- **Slip model** (replaceable, `MF52.SlipModel.BaseSlipModel` base):
  - `NoSlip` — zeros, for kinematic rigs.
  - `KinematicSlip` — algebraic, with a `blend = V/(V+V_min)` low-speed fade and clamps
    (`|κ| ≤ 2`, `|α| ≤ 1.2 rad`).
  - `TransientSlip` — PAC2002 relaxation with deformation states `u`, `v`; relaxation lengths
    `σ_κ`, `σ_α` from `PTX1..3`, `PTY1..2`, `PKY3`, falling back to 0.5 m when the coefficients
    are zero. Consistent initial equations for `u`, `v`. **This is what the standard vehicle uses.**
- **MF5.2 evaluation** (`MF52.Eval`): unpacks the coefficient record once, clamps evaluation load
  to `[FZMIN, ∞)` and rescales the result by `loadScale = Fz/FzEval` so forces go smoothly to zero
  at lift-off, calls pure-slip then combined-slip evaluators for Fx, Fy, Mx, My, Mz, returns
  pneumatic trail `t` and residual arm `s`, then applies the Z-up sign flip.
  - `PureSlip/`: `FxPureEval`, `FyPureEval`, `MxPureEval`, `MyPureEval`, `MzPureEval`
  - `CombinedSlip/`: same five, combined
  - Records mirror the layout exactly (`FxPureRecord` … `MzCombinedRecord`, plus
    `RelaxationRecord` and `SetupRecord`).

Contact patch normal load is read from the frame: `Fz = noEvent(max(0, cpFrame.f[3]))`.
Inclination angle `γ = asin(e_zw[2])`. Slip velocities are computed from the ground-projected
wheel basis (`e_xg`, `e_yg`) and the absolute wheel-centre velocity sensor.

### 4.3 Powertrain (EV stack)

```
BatteryPack ──HV──► InverterDC ──HV──► Motor ──shaft──► FixedRatioTransmission
                                                              │
                                            RearFinalDriveDifferential
                                              ├─ IdealGear (finalDriveRatio)
                                              ├─ diffInputRotor (J)
                                              ├─ Differential1D  or  LockedDifferential1D
                                              └─ left/right halfshaft SpringDampers
                                                     └──► rear wheel hubs
```

- **`EnergyStorage.BatteryPack`** wraps `Internal.TheveninBatteryPack` (Ns × Np cells, SOC state).
- **`PowerElectronics.InverterDC`** enforces `P_max_mot`, `P_max_reg`, `V_dc_max`.
- **`ElectricDrives.Motor`** wraps `Internal.PowerLimitedMotor`: an EMRAX-228-shaped envelope
  model with peak/continuous torque, current limit via `Kt`, peak power, a free-run loss table
  vs. rpm, constant motoring/regen efficiencies, and a `peakTime` timer that ramps peak limits
  down to continuous.
- **`Transmissions.FixedRatioTransmission`** — thin adapter over `Internal.FixedRatioGear`.
- **`Drivelines.Internal.Differential1D`** — a proper regularised LSD:
  - open kinematics `φ_in = (φ_L + φ_R)/2`
  - `T_lock_capacity = min(T_preload + 0.5·lockFraction·|T_in|, T_capacity_max)` with separate
    accel/decel lock fractions selected by `sign(driveSideTorqueSign·T_in)`
  - Gaussian blend between static and kinetic capacity across `w_transition`, `tanh` slip
    direction, plus small viscous term
  - publishes `lockingValue` and `torqueBiasRatio` diagnostics
  - `LockedDifferential1D` is the spool alternative, selected by `diff_lockedKinematics`
    (conditional component, so the unused one is removed at translation).
- **`Engines.SimpleICEngine`** exists as an alternative prime mover but is not in the standard stack.

### 4.4 Controllers and driver

- **`Controllers.Internal.VCUCore`** — the small piece: enable gating, torque clamp between
  `-regen_limit` and `tau_max`, near-zero-speed regularisation, `P_req = τ · ω_eff`.
- **`Controllers.VCU`** — the VehicleInterfaces controller adapter. Subscribes to
  `driverBus` (steering, pedals, R2D), `chassisBus.vehicleSpeed`, `batteryBus.{voltage,current}`,
  `electricMotorBus.speed`. Owns a `LimPID` speed controller and the regen/mechanical-brake blend
  (`regenBrakeBlend`). Publishes `electricMotorControlBus.{powerRequest, limitedTorqueCommand,
  regenTorqueLimit, vcuActive}`, `drivelineControlBus.driveTorqueCommand`, and
  `brakesControlBus.mechanicalBrakeTorqueRequest`.
  Three `enablePTN*SpeedControl` booleans decide whether the PI or the driver pedals drive each path.
- **`Controllers.StandardVCU`** — extends VCU with the manoeuvre generator. Four modes:

  | `useMode` | Manoeuvre |
  |---|---|
  | 0 | Open-loop **ramp steer** — integrates handwheel at `handwheelRampRate` until any tire normal load falls below `tireNormalLoadMin`, then smoothly rolls the rate to zero. |
  | 1 | Open-loop **sine steer** (`steerAmp`, `steerFreq`). |
  | 2 | **Step steer** (`frRampSteerHeight` over `frRampSteerDuration`). |
  | 3 | Closed-loop **steady-state Ay** — ramps `targetAy`, sampled PI on handwheel angle to drive measured `accY` to target, first-order lag on the command, handwheel limit. |

- **`DriverEnvironments.Internal.Driver`** — minimal intent publisher: steering angle, accelerator,
  brake onto `driverBus`, and a `Rotational.Sources.Position` driving the steering-wheel flange.
  `EVDriveByWire` adds direct EV torque/regen/R2D pins; `AutomaticDriveByWire` adds gear commands.

### 4.5 Aero and atmosphere

`Aero.Interfaces.Base` subscribes to `chassisBus.rideHeight_1..4` and the BobLib `atmosphereBus`
(density, wind, temperature, humidity, pressure), computes relative air velocity from the sprung
chassis frame, and applies `force`/`torque` through `Mounts.RigidMount` (a `FixedTranslation` by
`mountOffset`) into a `WorldForceAndTorque`.

`Aero.CFDAeroMap` averages front and rear ride heights, does a **clamped** bilinear lookup
(`Internal.Bilinear2D`, no extrapolation — saturates at the grid edges) on 5×5 CFD tables for
drag, downforce, Mx, My, Mz, then scales by `(ρ/ρ_ref)·(V/V_ref)²`.

`Atmospheres.ConstantAtmosphere` is a VehicleInterfaces atmosphere that also publishes onto the
BobLib `AtmosphereBus`; density is derived as `p/(R·T)`.

### 4.6 Brakes

`Chassis.Brakes.BasicVCUBrakes` — takes `mechanicalBrakeTorqueRequest` off the bus, clamps to
`[0, maxTorque]`, splits by a `frontBrakeBias` bias-bar fraction (0.55 default), halves per corner,
and applies torque opposing wheel spin with a `wRegularization` linear band around zero. It
publishes the four wheel speeds back on `brakesBus`. Explicitly no hydraulics, no ABS, no thermal,
no static hold.

---

## 5. Car definition and hardpoints

### 5.1 Where the data lives

`BobLib.Records.VehicleDefn` holds **9 complete cars**, one per suspension-architecture pair:

```
EVBatInvMotDiff_<front>_<rear>Record   where <front>,<rear> ∈ {DWDirect, DWBC, DWBCStabar}
```

Every one binds the same powertrain (battery/inverter/motor/differential) and differs only in the
axle record type. The reference/default car is
**`EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord`** — bellcrank + ARB at both ends.

Per-subsystem record *types* live in `Records.VehicleRecord.<Domain>`, mirroring the public
domain packages exactly.

### 5.2 Record composition of a car

```
EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord
├─ pFrAxleDW / pRrAxleDW      AxleDW_BC_StabarRecord   bellcrank, shock, spring/damper tables
├─ pFrStabar / pRrStabar      StabarRecord             bar geometry + rate
├─ pFrPartialWheel / pRr…     PartialWheelRecord       R0, rim, static toe/camber
├─ pFrRack / pRrRack          RackAndPinionRecord      pickup + C-factor
├─ pFrDW / pRrDW              WishboneUprightLoopRecord ← THE HARDPOINTS
├─ pFrAxleMass / pRrAxleMass  AxleMassRecord           4× MassRecord (unsprung, uca, lca, tie)
├─ pFrTire1DOF_YParams        Wheel1DOF_YRecord        wheelJ
├─ pFrTire1DOF_ZParams        Wheel1DOF_ZRecord        wheelC, wheelD
├─ pFrTireModel / pRrTireModel MF52Record              ~120 Pacejka coefficients per axle
├─ pBattery, pVCU, pInverter, pMotor, pDriveline       domain-owned powertrain records
├─ pBaseSprungMass, pDriverMass → pSprungMass = combineMassRecords({...})
├─ pTorsionalStiff            chassis torsion rate
└─ pAero                      CFDAeroMapRecord         5×5 CFD tables + ride-height refs
```

### 5.3 The hardpoints — `WishboneUprightLoopRecord`

Eight 3-vectors per corner, **left side only**, expressed in the chassis design frame (metres):

| Field | Meaning |
|---|---|
| `upperFore_i` | Upper control arm, **fore inboard** pickup |
| `upperAft_i`  | Upper control arm, **aft inboard** pickup |
| `lowerFore_i` | Lower control arm, fore inboard pickup |
| `lowerAft_i`  | Lower control arm, aft inboard pickup |
| `upper_o`     | Upper ball joint (outboard) |
| `lower_o`     | Lower ball joint (outboard) |
| `tie_o`       | Tie-rod outer pickup |
| `wheelCenter` | Wheel centre |

Derived geometry the models compute from these:
- Upper arm axis = `normalize(upperFore_i − upperAft_i)`; lower arm axis likewise.
- Kingpin axis = `normalize(upper_o − lower_o)` → the steering `Revolute` axis.
- The inboard *frame* is the **midpoint** of the fore/aft pair; the arm legs are drawn from there.
- `effectiveCenter = {wheelCenter[1], 0, wheelCenter[3]}` — the axle centreline reference that
  every `FixedTranslation` in the axle is measured from.

### 5.4 Reference car — actual numbers

**Front corner (left)**

| Hardpoint | x | y | z |
|---|---|---|---|
| upperFore_i | 0.1016 | 0.237744 | 0.2143252 |
| upperAft_i | −0.0680974 | 0.2356358 | 0.215138 |
| lowerFore_i | 0.1016 | 0.226314 | 0.08001 |
| lowerAft_i | −0.0762 | 0.226314 | 0.08001 |
| upper_o | −0.0092964 | 0.5420106 | 0.2679954 |
| lower_o | 0.0029972 | 0.562991 | 0.1139952 |
| tie_o | 0.0569976 | 0.546989 | 0.1522222 |
| wheelCenter | 0 | 0.6061108 | 0.199898 |
| rack leftPickup | 0.05715 | 0.2260092 | 0.1137158 |

**Rear corner (left)**

| Hardpoint | x | y | z |
|---|---|---|---|
| upperFore_i | −1.279144 | 0.2972308 | 0.2482342 |
| upperAft_i | −1.4993874 | 0.283845 | 0.2434336 |
| lowerFore_i | −1.3142214 | 0.283464 | 0.086868 |
| lowerAft_i | −1.4998192 | 0.2835148 | 0.0872236 |
| upper_o | −1.5540736 | 0.5267706 | 0.29464 |
| lower_o | −1.55448 | 0.57658 | 0.116078 |
| tie_o | −1.45796 | 0.5823966 | 0.2143506 |
| wheelCenter | −1.5494 | 0.6061108 | 0.199898 |
| rack leftPickup | −1.3763498 | 0.2897124 | 0.1700022 |

Note the rear "rack" is a static toe link mount — `Chassis_LockRrSteer` grounds the rear pinion.

**Bellcrank / shock (`AxleDW_BC_StabarRecord`)**

| | Front | Rear |
|---|---|---|
| `bellcrankPivot` | {−0.04214, 0.25075, 0.37001} | {−1.39887, 0.29230, 0.10160} |
| `bellcrankPivotAxis` | {0.95755, −0.26587, −0.11143} | {0.88796, 0.30271, 0.34625} |
| `bellcrankRodPickup` | {−0.01449, 0.34841, 0.37461} | {−1.41268, 0.35197, 0.08484} |
| `bellcrankShockPickup` | {−0.01103, 0.34554, 0.41126} | {−1.43801, 0.36137, 0.14160} |
| `bellcrankStabarPickup` | {−0.02901, 0.29714, 0.37220} | {−1.41347, 0.31058, 0.12307} |
| `rodToLower` | true (pushrod off LCA) | false (off UCA) |
| `rodMount` | {0.00676, 0.52561, 0.13447} | {−1.53509, 0.50331, 0.26648} |
| `shockMount` | {−0.02067, 0.24785, 0.56146} | {−1.50192, 0.28885, 0.36890} |
| spring rate | 26 269 N/m | 43 782 N/m |
| `springFreeLength` | 0.1997 m | 0.2634 m |
| damper | ±850 N @ ±1 m/s (linear) | ±1300 N @ ±1 m/s |
| ARB `barRate` | 257.62 N·m/rad | 535.47 N·m/rad |
| ARB `leftArmEnd` | {−0.03683, 0.26670, 0.11598} | {−1.43001, 0.30321, 0.40548} |
| ARB `leftBarEnd` | {−0.10665, 0.26670, 0.11811} | {−1.39252, 0.30321, 0.41224} |

**Derived vehicle geometry**

| Quantity | Value |
|---|---|
| Wheelbase | **1.5494 m** |
| Track (front = rear) | **1.21222 m** |
| Tire unloaded radius `R0` | 0.2045 m (≈16.1 in dia.) |
| Rim radius / width | 0.1278 m / 0.1789 m |
| Static camber / toe | 0 / 0 |
| Rack C-factor | 0.0889 m/rev (both axles) |
| Wheel spin inertia | 0.02 kg·m² |
| Chassis torsional stiffness | 300 000 N·m/rad |

**Masses**

| Item | m [kg] | rCM [m] |
|---|---|---|
| Base sprung | 160.640 | {−0.920, 0, 0.250} |
| Driver | 65.771 | {−0.5418, ~0, 0.3958} |
| **Combined sprung** | **226.411** | {−0.81015, 0, 0.29235} |
| Front unsprung (per side) | 7.8227 | {−0.0061, 0.6016, 0.1980} |
| Front UCA / LCA / tie | 0.5578 / 0.5183 / 0.1346 | — |
| Rear unsprung (per side) | 7.3522 | {−1.5408, 0.5999, 0.2011} |
| Rear UCA / LCA / tie | 0.3493 / 0.4631 / 0.1329 | — |
| **Total (both sides)** | **261.073** | {−0.80027, 0, 0.27962} |

Full 3×3 inertia tensors are given for every body.

**Tire model** — Pacejka MF5.2, `FNOMIN = 650 N`, `FZMIN = 100 N`, `FZMAX = 1800 N`, identical
coefficient sets front and rear. Notable: relaxation coefficients `PTX1..3` / `PTY1..2` are all
zero, so `TransientSlip` falls back to the 0.5 m default relaxation lengths. Combined-slip Mx/My
records are left at defaults.

**Powertrain**

| | Value |
|---|---|
| Battery | 140S4P, SOC start 1.0 |
| Inverter | 124 kW motoring / 124 kW regen, 588 V max |
| Motor | EMRAX-228-shaped: 220 N·m peak / 130 N·m cont, 6500 rpm peak, 360 A peak / 180 A cont, Kt 0.61, 124 kW peak, 75 kW cont, η 0.96/0.95, rotor J 0.02521 |
| Final drive | 3.31 |
| LSD | preload 20 N·m, 35 % accel / 15 % decel lock, 1000 N·m capacity |
| Halfshafts | 15 000 N·m/rad, critically damped (34.64 N·m·s/rad) |
| VCU | τ_max 220 N·m, regen limit 220 N·m |

**Aero** — reference speed 15 m/s; ride-height grids of 5 front × 5 rear breakpoints
(35.6–106.7 mm front, 41.9–125.7 mm rear); drag ≈ 154–180 N, downforce ≈ 163–352 N, My ≈ −307 to
−557 N·m at reference speed. Mx and Mz tables are all zero.

---

## 6. How it all plugs together

### 6.1 The redeclare chain (this is the core "relation")

```
VehicleSim
  └─ extends Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar
       └─ extends Templates.Vehicle.BaseVehicleSim          ← owns wiring + termination monitors
            ├─ replaceable record VehicleRecord = Records.VehicleDefn.…Record
            ├─ replaceable chassis      constrainedby VehicleInterfaces.Chassis.Interfaces.TwoAxleBase
            ├─ replaceable battery      constrainedby VehicleInterfaces.EnergyStorage.Interfaces.Base
            ├─ replaceable vcu          constrainedby BobLib.Controllers.StandardVCU
            ├─ replaceable inverter     (BobLib.PowerElectronics.InverterDC)
            ├─ replaceable motor        constrainedby VehicleInterfaces.ElectricDrives.Interfaces.Base
            ├─ replaceable transmission constrainedby VehicleInterfaces.Transmissions.Interfaces.Base
            ├─ replaceable driveline    constrainedby VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase
            ├─ replaceable brakes       constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase
            ├─ driverEnvironment        (Internal.Driver)
            ├─ inner replaceable road / atmosphere / world
            └─ protected aeroModel      (Aero.CFDAeroMap)
```

And inside the chassis:

```
Chassis_DWBCStabar_DWBCStabar               ← binds pVehicle, computes initial pose
  └─ extends Chassis_LockRrSteer
       └─ extends ChassisBase               ← VehicleInterfaces TwoAxleBase adapter
            │                                  4 wheel hubs, chassisFrame, steeringWheel, controlBus
            ├─ cgFixed → cgFreeMotion → chassisFrame     ← the 6-DOF vehicle state (enforceStates)
            ├─ fixedContactPatch_1..4 → GroundPhysics ground_1..4 → detailedChassis.frame{FL,FR,RL,RR}
            └─ replaceable detailedChassis
                 └─ DetailedChassis_LockRrSteer : DetailedChassisBase
                      ├─ replaceable frAxleDW   (FrAxleDW_BC_Stabar)
                      ├─ replaceable rrAxleDW   (RrAxleDW_BC_Stabar)
                      └─ replaceable spaceFrame (FrameCompX)
```

`Chassis_DWBCStabar_DWBCStabar` is the only place where record data is bound to models: it
redeclares tires as `MF52Tire` with `Wheel1DOF_Y` + `TransientSlip` on all four corners, wires
every hardpoint / mass / tire-coefficient record, and computes:

- `chassisReferencePosition = pVehicleCG` (mass-weighted CG, used as the initial world pose)
- `contactPatchPosition_1..4` = `wheelCenter + R(γ_static, 0, α_static)·{0,0,−R0}`, mirrored for
  the right side
- `frontLeft/…RideHeightOffset` = aero ride-height reference minus the axle-centre reference

### 6.2 Bus architecture (publish/subscribe)

Everything rides one `VehicleInterfaces.Interfaces.ControlBus`, connected to every subsystem.
Nobody wires signals point-to-point; each subsystem *publishes what it owns* through
`RealExpression`/`BooleanExpression` blocks and *subscribes* through protected `RealInput` taps.

| Sub-bus | Publisher | Fields |
|---|---|---|
| `driverBus` | `Internal.Driver` (+ `EVDriveByWire`) | `steeringWheelAngle`, `acceleratorPedalPosition`, `brakePedalPosition`, `inverterEnable` |
| `chassisBus` | `ChassisBase` | `rideHeight_1..4`, `vehicleSpeed`, `Fz_1..4`, `bodyAcceleration_2` |
| `batteryBus` | `BatteryPack` | `voltage`, `current` |
| `electricMotorBus` | `Motor` | `speed` |
| `transmissionBus` | `FixedRatioTransmission` | `outputSpeed` |
| `drivelineBus` | `RearFinalDriveDifferential` | motor-side speed, diff input speed, halfshaft torques |
| `brakesBus` | `BasicVCUBrakes` | `wheelSpeed_1..4` |
| `electricMotorControlBus` | `VCU` | `powerRequest`, `limitedTorqueCommand`, `regenTorqueLimit`, `vcuActive` |
| `drivelineControlBus` | `VCU` | `driveTorqueCommand` |
| `brakesControlBus` | `VCU` | `mechanicalBrakeTorqueRequest` |
| `AtmosphereBus` (BobLib-owned, separate) | `ConstantAtmosphere` | wind, density, temperature, humidity, pressure |

The aero model is the clearest example of the pattern: it reads chassis-owned ride heights off
`controlBus.chassisBus` and atmosphere-owned density/wind off `atmosphereBus`, and never touches
either publisher directly.

### 6.3 State selection and numerics (deliberate choices)

- Vehicle 6-DOF: `cgFreeMotion` with `enforceStates = true`, `useQuaternions = false`,
  all starts fixed, `v_rel_a[1] = initialLongitudinalVelocity`.
- Suspension corner state: `lowerJoint_i` revolute, `StateSelect.always`.
- Bellcrank revolute: `StateSelect.prefer`.
- Chassis torsion revolute: `StateSelect.always`.
- Wheel vertical prismatic in `Wheel1DOF_Y`: `StateSelect.never` (rigid).
- Halfshaft `SpringDamper.phi_rel`: `StateSelect.never`.
- `nominal` attributes are set nearly everywhere (`phi(nominal=0.05)`, `w(nominal=1)`,
  `pen(nominal=0.01)`, `f_z(nominal=1000)`) — the library is clearly tuned for DASSL scaling.
- OM flags baked into the entry points:
  `--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection
  -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000
  --generateDynamicJacobian=none`, solver `dassl`, `jacobian=internalNumerical`.
- `initial equation` in `ChassisBase` spins all four wheels to `v/R` at t=0.

### 6.4 Manoeuvre termination monitors (`BaseVehicleSim`)

`VehicleSim` doesn't just run for 10 s — it terminates on physically meaningful conditions:

| Condition | Trigger |
|---|---|
| Tire lift | any `Fz ≤ tireLiftTerminateLoad` (75 N) after steer start |
| Spinout | `\|sideslip\| ≥ 20°` held for 20 ms |
| Lateral-gain loss | local `dAy/dδ` drops ≥ 20 % below the gain latched at Ay = 4 m/s², held 50 ms |
| Open-loop QSS | `\|d(yawRate)/dt\|` and `\|d(handwheel)/dt\|` below tolerance for 0.1 s after the ramp ends |
| Steady-state Ay reached | Ay, speed, yaw-rate derivative, sideslip rate, roll rate, handwheel rate all inside tolerance for 0.1 s |
| Timeouts | 3 s after ramp end (open loop), 20 s after target ramp (closed loop) |

All of them use sampled finite differences at `linearitySlopeSamplePeriod = 0.1 s` with `pre()`
guards — no `der()` chains through the plant.

### 6.5 Four-post rig (`BaseFourPostSim`)

No vehicle body, no gravity (`world(g = 0)`). Two axles are held by `ChassisActuator`s that
impose prescribed **heave** and **roll** at each axle reference and measure jacking force via a
`CutForce`. Contact patches are held by `ContactPatchFixture` (x/y prismatics + spherical, z
grounded, with a `Disc` offset revolute to dodge a singular pose). `ContactPatchForceActuator`s
inject scripted Fx/Fy.

The scripted sequence is 118 s of 1 s steps: heave sweep −1→+1 over t = 2…56, then a roll sweep
−1→+1 over t = 63…117, with Fx pulses during the heave phase and Fy pulses during the roll phase.
Heave is held at zero through the roll phase (that was the 0.1.1 fix).

Outputs are collected into `FourPostEvalRecord` per axle: camber, toe, caster, KPI, mechanical
trail, scrub radius, spring lengths, Fz, ARB angle, jacking force, heave, roll, Fx, Fy.
Caster/KPI/trail/scrub are computed geometrically by intersecting the kingpin line with the
ground plane — a real K&C extraction, not a canned formula.

---

## 7. Test and CI infrastructure

`Tests/BobLibTest` is a **separate Modelica library** with 45 `.mo` files mirroring the BobLib
tree (`TestVehicle/TestChassis/TestSuspension/TestLinkages/TestBellcrank2.mo`, etc.), plus
`Regression/MF52PureSlipSmoke` and `Regression/VehicleSimAnimationOn`.

Six pytest modules drive OpenModelica through OMPython:

| Module | Checks |
|---|---|
| `test_modelica_linter.py` | pure-Python; formatting rules (120 col, blank-line layout, comma/equals spacing, long-call splitting) |
| `test_modelica_translation.py` | every standard model + every BobLibTest fixture translates; equation counts enforced |
| `test_modelica_initialization.py` | initialises each fixture, hashes variable names and values, compares to `modelica_initialization_baseline.csv` (26 models, incl. `VehicleSimAnimationOn` with 1018 variables) |
| `test_modelica_physics_validation.py` | simulates and compares named signals to `modelica_physics_baseline.csv` (12 assertions), plus runtime budgets from `modelica_runtime_baseline.csv` scaled by `BOBLIB_RUNTIME_SCALE` (default 4×) |
| `test_modelica_regression.py` | signal-level regressions |
| `test_boblib_modelica.py` | structural checks |

Each model is a separate pytest node id, so a failure names the exact model.

**CI split** (`.github/workflows/ci.yml`): pushes and PRs run only Python lint/tests, Modelica
formatting, and `make modelica-smoke`. The full gate (translation + initialization + physics +
regression) is `workflow_dispatch` with `full_modelica: true`, or locally via `make ci` / `make test`.
Version tags deliberately use the *light* gate.

---

## 8. Notable design decisions worth knowing

1. **No custom connectors.** One expandable connector for atmosphere; everything else is
   VehicleInterfaces or MSL. This is what makes the "swap any subsystem" claim real.
2. **Left-only records + automatic mirroring.** You define one side; `AxleDWBase` mirrors
   positions with `Vector.mirrorXZ` and inertia tensors with `Tensor.mirrorXZ`. Halves the data
   entry and makes asymmetry impossible by construction (which is also a limitation).
3. **Analytic loop closure.** `JointUSR` for the upper-wishbone/upright loop means OM does not
   need to iterate that constraint — a big part of why a full 4-corner MultiBody car is tractable.
4. **Event-free everywhere.** Ground contact, tabular spring/damper signum, brake direction,
   slip blending, LSD lock — all use smooth regularisations rather than `if`/events.
5. **Load-scaled tire forces.** `Eval` evaluates at `max(Fz, FZMIN)` then rescales by
   `Fz/FzEval`, which gives a clean, differentiable path to zero force on wheel lift instead of
   an MF blow-up at low load.
6. **Manoeuvre termination as physics.** `VehicleSim` ends on lift / spinout / gain-loss / QSS
   rather than a fixed clock — the templates are designed as *tests*, not animations.
7. **`headless` flag** propagated as `inner`/`outer` to disable all animation geometry for
   batch runs.

---

## 9. Inconsistencies found

Ordered roughly by impact.

### F1 — Vehicle CG/total mass counts only one side of each axle
`BobLib/Chassis/Chassis_DWBCStabar_DWBCStabar.mo:14-60` (and the same block in `Chassis_DW.mo:35-80`)

`pTotalMass` sums `pFrAxleMass.{unsprung,uca,lca,tie}` and `pRrAxleMass.{…}` **once each**. But
`AxleDWBase` instantiates a left *and* a mirrored right body from those same records, so the real
vehicle has two of each.

Verified numerically:

| | as coded | correct |
|---|---|---|
| total mass | 243.742 kg | 261.073 kg (−6.6 %) |
| CG x | −0.80486 | −0.80027 |
| **CG y** | **+0.04115 m** | **0** |
| CG z | 0.28553 | 0.27962 |

Two consequences: the CG y is 41 mm off-centre on a laterally symmetric car (an obvious tell),
and since `pVehicleCG` is used as `chassisReferencePosition`, the initial ride height / static
contact penetration is set by a ~6 mm error in CG z. It does not corrupt the dynamics (the
`Body` masses themselves are correct), but the initial pose is wrong and every initialization
baseline is anchored to it.

### F2 — The standard four-post runs the ARB car with zero bar rate
`BobLib/Experiments/Standards/Templates/FourPost/FourPostSim_DWBCStabar_DWBCStabar.mo:14,18`
(and every other `FourPostSim_*Stabar*` template)

The four-post templates pass ARB geometry through but hard-code `barRate = 0`, while the
vehicle templates pass `barRate = pVehicle.pFrStabar.barRate`. So `FourPostSim` — the flagship
K&C entry point for a car that has 257/535 N·m/rad bars — reports roll stiffness with no bar
contribution. If that is intentional (bar removed for K&C), it needs a comment and probably a
parameter; right now it looks like a placeholder that never got wired.

### F3 — `linkDiameter` / `jointDiameter` at the template level are dead
`BaseVehicleSim.mo:9,11` → `ChassisBase.mo:13,14` → `DetailedChassisBase.mo:9,10`

All three declare `inner parameter SI.Length linkDiameter = 0.020` / `jointDiameter = 0.030`.
The innermost `inner` wins for its subtree, so the `outer` lookups in `AxleDWBase`, `FrameBase`
and `PartialWheel` resolve to `DetailedChassisBase`'s hard-coded values. Changing the top-level
`Setup/Animation` parameters has no effect on any suspension geometry. (`headless` is handled
correctly — only `ChassisBase` declares it `inner`, and `BaseVehicleSim` binds it explicitly.)

### F4 — `staticAlpha` / `staticGamma`: degrees vs radians disagreement
`Records/…/Tire/Templates/PartialWheelRecord.mo:14-17` vs
`Tires/TirePhysics/Templates/PartialWheel.mo:35` vs `Chassis_DWBCStabar_DWBCStabar.mo:84+`

The record types them `SI.Angle` but documents "in DEGREES". Two consumers disagree:

- `PartialWheel.toHub` passes them **raw** into `FixedRotation.angles` (which is `SI.Angle`,
  i.e. radians).
- `Chassis_*` and `BaseFourPostSim` convert with `*pi/180` when computing contact-patch positions.

Both values are `0` in every shipped record, so this is latent. Any non-zero static camber or toe
gives a wheel attitude and a contact-patch position that disagree by a factor of 57.3.

### F5 — Tire vertical rate in the record is unused; ground stiffness is hard-coded
`Records/…/Wheel1DOF_ZRecord`, set to `wheelC = 98947`, `wheelD = 115.844`, versus
`ContactMechanics/GroundPhysics.mo:19-20` `c = 100000`, `d = 750`.

The standard chassis redeclares `Wheel1DOF_Y` (rigid vertical), so vertical compliance comes
entirely from `GroundPhysics`, and no `GroundPhysics` instance anywhere is ever parameterised.
Stiffness is close by luck (100 000 vs 98 947), but damping is **6.5× the record value**. Tire
vertical damping is a real parameter for four-post and ride work; right now it can't be set from
a vehicle record.

### F6 — `rodPickup` / `shockPickup` / `stabarPickup` are dead parameters
`Records/…/Suspension/AxleDW_BC_StabarRecord.mo` (and `AxleDW_BCRecord`)

These Integer "pickup mapping" indices are set in all 9 vehicle records (front `2/3/1`, rear
`1/2/3`, etc.) but no model reads them — `FrAxleDW_BC_Stabar` hard-wires
`pickup_1 = bellcrankRodPickup`, `pickup_2 = bellcrankShockPickup`, `pickup_3 = bellcrankStabarPickup`.
Either the ordering logic was never implemented or it was removed and the parameters left behind.
Confusing, because the values differ front-to-rear as if they mattered.

### F7 — Unused parameters in `ShockLinkage`
`Chassis/Suspension/Linkages/ShockLinkage.mo:11-18, 30-33`

`r_a`, `r_b`, `n_a`, `n_b`, `linkDiameter`, `jointDiameter` are declared and populated by both
axle models — with real geometry and normalised axes — but none appear in any equation. The model
is just `LineForceWithMass` + spring + damper. Also
`parameter Boolean fixedRotationAtFrame_a = false annotation(tab = "Advanced")` is a malformed
annotation (should be `Dialog(tab = "Advanced")`).

### F8 — Wrong unit types on the tabular spring/damper data
`AxleDW_*Record.mo`, `ShockLinkage.mo`, `TabularSpring.mo`, `TabularDamper.mo`

- `springTable` is typed `SI.Position[:,2]` in the records and `SI.TranslationalSpringConstant[:,2]`
  in the models — it is actually `[m, N]`.
- `damperTable` likewise, actually `[m/s, N]`.
- In the records both tables sit under `Dialog(group = "Geometry")`.
- `springFreeLength`'s description string is also filed under Geometry.
- Typo in the record description: `"[dx1, F1; dx1, F2; ...]"` — second `dx1` should be `dx2`.

Harmless to the solver, but it makes the GUI and any unit-checking tool wrong.

### F9 — Record documentation describes fields that aren't there
- `AxleDW_BC_StabarRecord`: "combines bellcrank pickups, spring and damper data, **stabilizer-bar
  geometry, steering rack data, and double-wishbone hardpoints**" — it contains none of those last
  three; they're separate records.
- `AxleDW_DirectRecord`: "contains **shock, rack, wheel, mass, and hardpoint data**" — it contains
  shock data only.
- Frame-of-reference wording is inconsistent: `WishboneUprightLoopRecord`, `StabarRecord` and
  `RackAndPinionRecord` say "expressed in chassis frame", while `AxleDW_*Record` says "resolved in
  world frame" for what is the same design frame.

### F10 — Conflicting defaults between a model and its record
`Drivelines/RearFinalDriveDifferential.mo:22` has `diff_lockedKinematics = true`;
`Records/…/RearFinalDriveDifferentialRecord.mo:12` has `= false`. The templates always pass the
record value, so the effective behaviour is open+LSD — but instantiating the driveline standalone
silently gives you a spool. Same file: the model's halfshaft damping defaults hard-code
`2*sqrt(c*0.02)` while the record exposes a proper `halfshaftLeftJEquivalent` parameter.

### F11 — `FrameCompX` takes the sprung mass record twice
`Chassis/Body/FrameCompX.mo:9` declares `pSprungMass` while inheriting `pSprung` from `FrameBase`.
`Chassis_DWBCStabar_DWBCStabar.mo` sets both to the same value. `pSprungMass` is used only for the
critical-damping estimate. One of them should go.

### F12 — Naming: "final drive ratio" isn't in the final drive
`BaseVehicleSim.mo:205,211`: `transmission(gearRatio = pVehicle.pDriveline.finalDriveRatio)` and
`driveline(finalDriveRatio = 1, ...)`. Numerically consistent, and there's a defensible reason
(keep the reduction in the VI transmission slot), but a model called
`RearFinalDriveDifferential` parameterised with `finalDriveRatio = 1` while a
`FixedRatioTransmission` carries 3.31 will mislead anyone reading the diagram.

### F13 — Smaller items
- `Utilities/…/Actuators/ContactPatchFixture.mo:36-38`: an `Icon(graphics = {...})` annotation is
  nested **inside the `connect(...)` annotation**, and a second `Icon` appears in the class
  annotation. The first is dead.
- `ElectricDrives/Internal/PowerLimitedMotor.mo:9-11`: the `P_elec` description string is
  multi-line and contains a Unicode minus (`−`) rather than ASCII.
- `ChassisActuator.mo:60`: uses `Parts.FixedRotation(r = axleRef)` with no rotation where
  `FixedTranslation` is the idiomatic component.
- `FrAxleDW_BC_Stabar.mo`: the `SphericalSpherical` droplinks don't honour `headless`
  (`animation` left at default true), unlike everything else in the file.
- `DriverEnvironments/package.order` lists `Internal` last; every other domain lists it first.
- `Aero.Internal.Bilinear2D` clamps at the grid edges with no warning — bottoming out below
  35.6 mm front ride height silently returns the edge value.
- Physics baselines cover only 3 models (`MF52PureSlipSmoke`, `TestCFDAeroMap`, `TestVCU`).
  Neither `VehicleSim` nor `FourPostSim` has a signal-level physics baseline.
- `knowledge.md` (untracked, at repo root) documents an architecture that no longer exists.

---

## 10. Suggestions

**Correctness first**

1. Fix F1. Factor the CG/total-mass calculation into
   `Utilities.Mechanics.Functions.combineMassRecords` — that function already does mass-weighted
   CG with parallel-axis inertia. Build the axle contribution as
   `{unsprung, uca, lca, tie}` plus their `mirrorXZ` twins and hand the whole list to it. The
   130 lines of hand-expanded arithmetic duplicated across `Chassis_DW.mo` and
   `Chassis_DWBCStabar_DWBCStabar.mo` collapse to about five, and the `CG_y = 0` result becomes
   structural. Add a regression assertion that `abs(pVehicleCG[2]) < 1e-9` for symmetric records.
2. Fix F4 before anyone runs a non-zero static-camber study. Cleanest: make the record store
   radians (drop "in DEGREES" from the description, keep `SI.Angle` with
   `displayUnit = "deg"`), and delete the `*pi/180` from the two consumers. That's the Modelica
   convention and it makes `SI.Angle` honest.
3. Decide what F2 should be. If the four-post is meant to run bar-off, rename the parameter or add
   `barRateScale` and default it to 1 with an explicit override; if it's a bug, wire
   `barRate = pVehicle.p*Stabar.barRate` and re-baseline.
4. Fix F3 by deleting the `inner` declarations in `ChassisBase` and `DetailedChassisBase` and
   passing `linkDiameter`/`jointDiameter` down explicitly the way `headless` already is. Right now
   there is a parameter in the GUI that does nothing.
5. Expose `GroundPhysics` `c`/`d` (F5). Add them to a small `ContactRecord` in the vehicle
   definition, or at minimum bind `ground_1..4` to `pFrTire1DOF_ZParams.wheelC/wheelD`. A 6.5×
   error in tire vertical damping matters for the four-post work this library is aimed at.

**Cleanup**

6. Delete F6 and F7 (dead parameters). If the bellcrank pickup indices are a planned feature,
   move them to an issue rather than shipping populated no-ops.
7. Fix the unit types in F8. `Real[:,2]` with a clear description is more honest than a wrong
   `SI.*` type. Add a `Dialog(group = "Spring")` / `"Damper"` instead of `"Geometry"`.
8. Sweep the record documentation (F9) — several `<html>` blocks describe an older, monolithic
   record layout. Standardise on one phrase for the design frame ("expressed in the chassis design
   frame") and use it everywhere.
9. Reconcile F10, F11, F12, and the F13 items. Most are one-line changes.

**Structure and coverage**

10. Add a physics baseline for `VehicleSim` and `FourPostSim`. Right now the only end-to-end
    protection for the flagship models is "does it translate and initialise to the same hashes."
    A handful of scalar assertions — steady-state Ay at a fixed handwheel, front/rear roll
    stiffness from the four-post, static corner weights — would catch exactly the class of bug in
    F1 and F2.
11. Consider a `Records.VehicleDefn` consistency test in Python: parse the nine records and assert
    that shared blocks (powertrain, tire coefficients, masses, aero) are byte-identical across
    them. Those nine files are ~500 lines each and largely duplicated; a single edit to a Pacejka
    coefficient currently has to be made nine times. Longer term the right fix is a base record
    that each variant extends and overrides only its axle fields.
12. Extend `modelica_linter.py` with two cheap structural rules: flag parameters declared and never
    referenced within a class (catches F6/F7), and flag `annotation(tab = ...)` outside a `Dialog`
    (catches the F7 malformation).
13. Delete or regenerate `knowledge.md`. A stale architecture document at the repo root is worse
    than none — it will be picked up by tooling and by anyone new.
14. Add `CONTRIBUTING.md` pointing at `AGENTS.md`'s package-boundary rules. Those rules are good
    and non-obvious, and they are currently only discoverable in an agent-oriented file.
