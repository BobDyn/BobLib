# Release-readiness scope

This stack prepares the Modelica library for review; it does not certify a release.
The foundation is the preserved pre-SS release candidate of 2026-10-20. Native/FMU export tooling and calibration tests are included. VehicleSimSS,
SS initialization hooks and GGV prototypes are excluded. The FMU exporter already merged in PR #30 is retained. Later GGV-related plant changes remain separate. The
existing dynamic maneuver controller and VehicleFMI Modelica boundary remain.

Layers separate package styling, calibration contracts, typed buses, mechanics,
brakes, suspension components, wheel compliance, aero, differential components,
drivers and electrical drive. The coupled vehicle API migration follows, then
fixture coverage, domain regression checks and CI/migration documentation. Two final tooling layers add configurable FMU export, then native export and
cross-runtime calibration verification. They each contain corresponding tests.

## Fresh isolated checks

- Six public entry points passed OpenModelica checkModel with balanced equations:
  VehicleSim, VehicleFMI, FourPostSim, KnCSteerable, KnCLocked and ManeuverFMI.
- 1,067 tests collected without dependencies on the excluded tooling.
- 613 focused import, routing, diagram, coverage, parameter-documentation,
  architecture and formatting-helper tests passed.
- The mounted-transmission regression passed with unique simulation output prefixes,
  fixing the previous filename/GUID collision.
- Full-tree Modelica formatting passed after an annotation-only MassRecord fix.

This is not a fresh full simulation-suite pass. The historical snapshot reported
1,302 passes and one fixture failure, including checks outside this stack; that
result cannot certify these extracted commits.

## Remaining before release

Run the full gate on final merged sources, justify baseline changes case by case,
complete OMEdit review of every model and relevant conditional variant, and validate
numerical behavior across supported architectures. Native/FMU artifact acceptance must use the final exact source identity; a single
architecture validation is not acceptance of the complete catalog.

The merged FMU-export PR #30 has been incorporated. Reconcile the unmerged
damper-table work against the new package and record paths; compatibility with
unmerged PRs is not established by these drafts.
