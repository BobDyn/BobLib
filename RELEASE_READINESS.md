# Release-readiness draft scope

This stack prepares the Modelica library for review; it does not certify a release.
The foundation is the preserved pre-SS release candidate of 2026-10-20. Python
native/FMU build and runtime tooling, VehicleSimSS, SS initialization hooks and GGV
prototypes are excluded. Later GGV-related plant changes remain separate. The
existing dynamic maneuver controller and VehicleFMI Modelica boundary remain.

Layers cover the coupled library migration and Modelica fixtures, regression and
visual checks, then CI/migration documentation. New Python files support tests
only; no Tools directory or runtime export runner is included.

## Fresh isolated checks

- Six public entry points passed OpenModelica checkModel with balanced equations:
  VehicleSim, VehicleFMI, FourPostSim, KnCSteerable, KnCLocked and ManeuverFMI.
- 1,059 tests collected without dependencies on the excluded tooling.
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
numerical behavior across supported architectures. Native/FMU artifact acceptance
belongs to the later tooling work and must use its exact source identity.

Reconcile existing FMU-export and damper-table PRs against the new package and
record paths. Compatibility with unmerged PRs is not established by these drafts.
