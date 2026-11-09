# Car-only SIL FMU

`BobLib.Experiments.Standards.VehicleSIL` is the vehicle plant for an external
firmware VCU. `VehicleFMI` retains its existing driver-input interface and internal
BobLib controller. They produce distinct artifacts.

## Build and check

Run from the repository root. Docker Desktop must be running with Linux containers.
The same Linux x86-64 image builds and exercises the FMU on Windows, macOS and Linux;
the resulting binary is intended to run inside that environment, not load into a
native Windows Python process. Apple Silicon uses x86-64 emulation.

```sh
docker build --platform linux/amd64 -f Tools/Dockerfile.sil -t boblib-sil:omc1.27.1 .
docker run --rm --network none --mount "type=bind,source=$PWD,target=/workspace" boblib-sil:omc1.27.1 python Tools/export_fmu.py --sil --rear-brakes Inboard --output build/sil --jobs 4 --reuse
docker run --rm --network none --mount "type=bind,source=$PWD,target=/workspace" boblib-sil:omc1.27.1 python Tests/sil_acceptance.py --fmu build/sil/BobLibSIL_DWBCStabar_DWBCStabar_RearInboard/BobLibSIL_DWBCStabar_DWBCStabar_RearInboard.fmu --output build/sil-acceptance --jobs 3
```

In PowerShell, substitute `${PWD}` for `$PWD` inside the mount string. Use a new
output directory for each acceptance run so previous evidence is preserved. Add
`--quick` for one six-second run; the full check also repeats it, compares 1 ms
with 3 ms exchange and 1e-6 with 1e-7 tolerance, and checks reverse motoring/regen
and disabled standstill. Nonzero exit means acceptance failed. CSV signals, FMI
messages, timing and a JSON verdict remain available after a solver failure.

`--reuse` skips compilation only when the source/wrapper hashes, architecture,
compiler, flags, dependency versions and build environment match. The archive
SHA-256 and embedded provenance must also match its sidecar manifest. A missing,
stale or corrupt artifact is rebuilt. `last-export.json` records elapsed time and
reuse status for the last successful export. Restore the artifact directory in
CI before using this option; an empty workspace still requires a cold build.

Acceptance defaults to one process. `--jobs 3` runs the independent repeat and
sensitivity profiles in up to three fresh processes after the baseline. Each
has its own FMU state and logs; no native runtime is shared between workers.
Choose the count to suit available CPU cores. This changes wall-clock scheduling,
not the simulation exchange interval. Acceptance checks also remain active under
Python `-O`.

The build retains calibration, source hashes, architecture choices, compiler
version, runtime corrections, FMU SHA-256 and phase timings. An embedded build
manifest travels with the FMU. Numeric physical calibration remains editable
before initialization; axle/brake/differential architecture is selected at build
time. This example explicitly selects rear inboard brakes and the shipped
stabilizer/clutch-LSD calibration. It does not establish which hardware
configuration or calibration is correct for a particular car.

## Boundary

`sil-interface.json` is version 1 of the actuator/sensor contract. Units and
causality are checked against the actual exported FMI metadata.

| Input | Meaning |
| --- | --- |
| `motorTorqueCommand` | Signed motor-axis N·m request; positive drives forward. Negative at forward speed requests regen. |
| `inverterEnable` | False requests zero drive power. Existing motor torque decays with its physical time constant. |
| `brakeCommand` | Mechanical brake actuator fraction [0,1], independent of motor torque and enable. |
| `steeringAngleCommand` | Steering-wheel radians, passed through the existing 50 Hz position filter. |

Outputs include signed motor rpm, four wheel angular speeds in rad/s, DC voltage,
DC current (positive discharge), signed longitudinal velocity and true speed
magnitude. Acceleration and drive diagnostics are available for grading. Extra
inherited vehicle outputs include tire loads, yaw rate, roll and lateral velocity.
Sensor outputs are ideal physical measurements. Ground truth is not a substitute
for the noisy or delayed estimate available to firmware.

The reusable exporter includes the OMC 1.27.1 numerical corrections already used
by BobLib's older FMU tests: recoverable nonlinear RHS evaluations and
solver-scale relation hysteresis. State-event handling stops with an error after
100 crossings in one exchange rather than looping indefinitely. Boolean enable
changes, as well as real actuator changes, invalidate the integrator's input
history. These are compiler/runtime corrections, not motor-physics retuning.

Direct torque also exposed a separate physical-model defect: a full pack and a
binary inverter voltage cutout could alternately allow and reject regen, leaving
the algebraic voltage/current solve without a solution. The inverter now tapers
DC capacity over `pVehicle.inverter.voltageDerateBand` (one volt by default) at
both voltage limits. Existing voltage limits are retained. Regen can supply
standby loss at the upper limit without charging the pack. This narrow band is
an averaged protection approximation, not measured inverter calibration. The
full-charge/low-voltage component regression and full-charge FMU launch cover it.

The existing averaged inverter/motor accepts power, so the boundary converts
torque to power using the motor's latched shaft direction and smooth near-zero
speed magnitude. It preserves physical efficiency, losses, saturation and torque
dynamics. Delivered torque is therefore different from the input request. This
retains BobLib's averaged-drive approximation, including its low-speed electrical
power approximation. There is no torque compensation, ideal torque source, VCU
power cap, pedal map, traction controller, launch ramp or regen blending in this
plant. The firmware owns those policies. The external controller must gate
low-speed regen; negative torque is not automatically suppressed at standstill.

## Connection to the harness

Use FMI 2 Co-Simulation. Initialize with enable false and zero torque/brake/steer
(or explicit scenario commands), then supply zero-order-held commands before each
`doStep`. Use integer simulation ticks, normally 1 ms, and run firmware control
every 3 ms in the future harness. The FMU's internal solver resolves continuous
physics; a 1 ms exchange does not promise real-time wall-clock performance.
Always stop and preserve diagnostics if an FMI operation fails. Create a fresh
instance for another scenario. Set physical parameters before initialization.

The external harness owns APPS1/2, brake-pressure-to-actuator calibration, RTD,
shutdown/HVC states, cell-voltage faults, CAN encoding, update rates, delay, drops
and firmware scheduling. Those are not synthesized as plant measurements here.
Pedal maps, power limiting, traction, launch, regen and state/fault gating can all
exercise this plant through the same boundary. Pedal plausibility and fault logic
also need scripted electrical inputs in the harness; there is no full hydraulic,
thermal, contactor or cell-level model in this boundary.

## Acceptance limits and scope

The scripted check requires launch/relaunch above 1 m/s, brake hold below 0.05 m/s,
zero requested power and motor target when disabled, negative DC current and a
speed decrease under forward regen, steering response, finite measurements and
positive voltage. Full acceptance requires repeatability and maximum signed-speed
differences below 0.25 m/s for the exchange/tolerance comparisons. These are fixed
engineering smoke limits, not VCU rule limits or a claim of track accuracy.
The main sequence keeps the shipped full-charge initial state. A separate test
requires full-charge regen to curtail charging without exceeding the voltage
limit. Reverse motoring/regen tests explicitly start at 80% charge so charging is
physically available; the chosen initial state is recorded in each run report.

For CI, build once and run `Tests/sil_acceptance.py` against that exact artifact;
`BOBLIB_SIL_FMU=/path/to/car.fmu python -m pytest Tests/test_sil_fmu.py` runs the same
full acceptance through pytest. The default unit suite checks boundary safeguards
without recompiling a full vehicle. No firmware harness, PR merge policy, car-log
fitting or tuning infrastructure is added here. Validation of one configuration
does not certify the whole architecture catalog.

## Verified on November 8, 2026

Linux x86-64 Docker, OpenModelica 1.27.1, MSL 4.1.0, VehicleInterfaces 2.0.2;
stabilizer front/rear axles, clutch LSD, front outboard/rear inboard brakes.

The exact accepted artifact is recorded in
`build/sil-acceptance-voltage-fix/acceptance.json` with CSV signals and per-run
logs beside it. FMU SHA-256:
`64e8e5d6313589b929374ae6614cd78fe0af2fd2ca0865bc669713932faaafa1`.
Source fingerprint:
`ef7ce1fd5ca20fb5b573d4588ebfdd35acc2a2d63e93cb5a8e7a8d765930cd74`.

| Check | Result |
| --- | --- |
| Full-charge launch, stop/hold, relaunch, disable, regen and steering | Passed; first launch 4.31 m/s, relaunch 6.54 m/s |
| Brake hold | Maximum signed speed magnitude 0.0068 m/s |
| Repeatability | All required outputs agree within 1e-8 absolute/relative tolerance |
| 3 ms versus 1 ms exchange | Maximum signed-speed difference 0.00082 m/s |
| 1e-7 versus 1e-6 solver tolerance | Maximum signed-speed difference 0.00040 m/s |
| Full-charge regen, disabled standstill, reverse motoring/regen | Passed |
| Drive saturation and braking with inverter disabled | Passed through pytest |
| Electrical voltage-cutoff regression | Original inverter equations fail; fixed equations pass natively and through FMI |
| Model Exchange launch comparison | Reaches 1 s successfully; the original cutoff failed at 0.633 s |

The six-second 1 ms run took 65.9 s including extraction/initialization; the
3 ms run took 54.4 s. The final full build took 335 s on this machine while
other checks were running. These measurements do not meet real-time execution
or establish a one-minute PR budget. Cache a pinned plant artifact separately
from firmware changes; further model/runtime performance work is outside this
change.

Parameter documentation, import, package/coverage, diagram-route, Python lint
and Modelica formatting checks passed. These are focused acceptance results,
not a full-library release certification. OMEdit visual inspection was started
but stopped by the user; rendered diagram review remains outstanding.

## Efficiency review, November 8

The reviewed build and its complete passing acceptance are recorded in
`build/sil-reviewed/` and `build/sil-reviewed-acceptance/acceptance.json`.
FMU SHA-256:
`d3886d9ce005702607f67cda1722be6f095916b7ebb67d995166f609bf7452e4`.
Source fingerprint:
`2615a3311efe7aa5acb3bf4330c99ada04397fd44d6fe02c33f765658b4a84b9`.

Profiling put about 93% of measured execution time in physics stepping. SIL
exports now compile physical code with `-O2`; metadata retains `-O0` to keep its
large initializer cheap to compile. Other export modes retain `-Os`. In a paired
six-second benchmark, `-O2` reduced wall time from 70.7 to 62.8 s (11.2%); all 14
logged outputs were identical between those compiler builds. The runner also
avoids resending unchanged actuator values. That reduced setter calls from about
12,000 to 10, but made little total-time difference; it changed solver bookkeeping
slightly, with a maximum signed-speed difference of 0.000652 m/s against the old
runner, well within the existing 0.25 m/s acceptance limit.

| Final reviewed check | Result |
| --- | --- |
| Cold export | 277.8 s |
| Verified reuse of that exact artifact | 4.4 s; archive hash unchanged |
| Six-second 1 ms baseline | 58.5 s |
| Complete acceptance with three profile processes | 165.8 s; passed, including under Python `-O` |
| Repeatability | All required outputs agree within 1e-8 absolute/relative tolerance |
| 3 ms versus 1 ms speed difference | 0.000620 m/s |
| Tighter-tolerance speed difference | 0.000921 m/s |
| Final drive saturation / disabled-inverter mechanical braking | Both passed |

The Python gate passed 226 checks (three opt-in runtime checks are skipped by
that gate and were exercised separately); 134 mirrored fixtures are accounted
for. No physical calibration, 1 ms default exchange or acceptance threshold was
relaxed. The plant remains slower than real time; the principal remaining cost
is the native physics solver, rather than Python signal packing.

## Branch checkpoint, November 9

The shared-exporter downstream gate completed with six passes: state-event
localization, homotopy early success, maneuver-driver export without a nonlinear
solver, requested solver tolerance, default vehicle calibration, and native/FMU
record-dependency recalculation. The local result is retained at
`build/downstream/shared-exporter.xml`.

This is a branch checkpoint, not whole-library or BobSim release approval. The
legacy driver-input VehicleFMI end-to-end suite still needs a successful run;
the earlier attempt failed during test-directory setup before simulation. Full
BobSim regression simulations and OMEdit visual review also remain outstanding.
BobSim's checked-out BobLib pin was not changed, and no regression baseline was
regenerated to conceal a failure. The inverter protection change affects shared
BobLib physics near its voltage limits and needs that broader review before
merging or updating downstream dependencies.
