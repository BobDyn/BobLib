# Agent Notes

These rules capture the current BobLib/BobLib architecture so
future agents preserve the package boundaries.

## Active work

- Read `MIGRATION.md` and `RELEASE_READINESS.md` for migration scope and remaining release work.

## BobLib Architecture

- Every package icon has a black, unfilled outer border matching its icon canvas,
  including functional, interface, Internal, record, and test packages. Preserve
  representative artwork; do not add this package border to model icons.

- Place each visible MultiBody `world` at the bottom-left of its diagram, using
  the standard 20-by-20 placement. Align diagram bounds to full major grid cells. Keep
  component symbols compact; clipping long contents is preferable to excessive
  block sizes. Prioritize clear routes, separated symbols, and readable names.

- Use direct imports (`import Modelica.Units.SI;`, then `SI.Length`), not
  redundant renaming imports. Group imports near the class header. Avoid aliases;
  if a genuinely ambiguous or unwieldy name requires one, keep it in a small,
  separate `// Import aliases` section and document its exception in the import
  regression test. Never alias SI. Follow the early hand-written chassis style:
  short section comments, grouped declarations, explicit readable modifiers,
  and no decorative comment banners or repeated visibility sections.

- Every local parameter declaration needs a meaningful Modelica description,
  including calibration records, derived parameters and test fixtures. State
  coordinate frames, table columns, units not conveyed by the type, and effects
  of switches. Order model settings by calibration, architecture/connectors,
  operating settings, initialization, animation and advanced controls. Preserve
  record field order (positional constructor API) and compiler-sensitive
  declarations before inherited consumers. Retain meaningful physical groups.
  Run `Tests/test_parameter_documentation.py` when adding or changing parameters.

- Public editable Boolean parameters must use `choices(checkBox = true)`,
  including records and test fixtures. Preserve Dialog/Evaluate settings and
  defaults. Do not redeclare non-replaceable VI/MSL parameters merely to change
  their widgets; inherited upstream controls are a documented exception.

- Disabled mechanical flange/frame connectors retain the editor's translucent
  inactive symbols. Their wiring must be hidden with `Line(visible = <condition>)`.
  Hide inactive implementation components with `Placement(visible = <condition>)`.
  Never reference a subsystem's protected VI flags from an enclosing assembly,
  including in annotations. Physical-pedal assemblies own their pedal wiring.
  Verify graphical instantiation as well as checkModel and rendered variants.

- Model-level parameter records live under `Records.VehicleRecord`; complete
  vehicle definitions live under `Records.VehicleDefn`. Name each model's record
  parameter for its owner (`pChassis`, `pAxle`, `pFrame`, `pTire`, `pWheel`,
  `pMotor`, `pInverter`, `pBattery`, `pVCU`, `pDriveline`, `pDifferential`,
  `pTransmission`, `pBrakes`, `pSlip`), never the generic `pCalibration`.
  Pass child records down from that parameter. Keep architecture selections,
  VI connector switches, and experiment settings separate.

- Keep `package.order` consistent: guide/tutorial first when present, then
  `Interfaces`, functional subpackages, `BaseClasses`, direct partial/base
  classes, concrete classes, and `Internal` last. Within a group, order related
  variants from simplest to most elaborate; use case-insensitive alphabetical
  ordering when no meaningful progression exists. Keep matching model/record
  families in the same progression (direct, bellcrank, bellcrank with stabar).
  At the library root, put `UsersGuide` and `Experiments` first, alphabetical
  vehicle domains next, and `Records`, `Utilities`, and `Icons` last. Preserve
  the user guide's narrative order. This is BobLib's VI-inspired convention,
  not a claim that all MSL packages use one universal order.
- Keep the first level of each public domain package aligned with the
  `VehicleInterfaces` contract. First-level models are the shared adapters or
  concrete insertion points used by experiments.
- Put BobLib physics, implementation details, templates, and helper models one
  level deeper than the public contract layer. Do not add duplicate interfaces
  or extra connectors when an existing `VehicleInterfaces` interface can be
  used directly.
- Domain-local implementation models should live in that domain's nested
  packages, usually `Internal`, `Templates`, `Actuators`, or another
  purpose-specific package. Avoid a second public access path for the same
  interface or physics model.
- Public vehicle subsystem adapters extend BobLib-owned VI-derived interface
  bases when they expose controlBus; these retain the VI physical contract and
  use BobLib.Utilities.Interfaces.ControlBus. Other adapters use the appropriate VI base. Internal helpers do not need to inherit a complete subsystem contract;
  use only the physical or signal interfaces required by their actual role.
- Prune obsolete models and pass-through inheritance layers when their callers
  have migrated. `Internal` is for active implementation helpers, not an archive.
- Package icons use native MSL role icons for interfaces, base classes, records,
  functions, examples, and utilities; vehicle domains use representative VI
  symbols and mechanical packages use their existing component symbols. Keep
  the BobDyn identity at the root. Copy only graphics when the source is a
  model: packages must not extend icon models. Retain upstream attribution and
  license files when adapting graphics.
- All `Internal` packages extend `Modelica.Icons.InternalPackage` directly;
  preserve the MSL glyph without a BobLib background or badge overlay. Add the
  same black package border used throughout the library.
- Brake placement is an experiment-level categorical choice. Axle `leftTorque`/
  `rightTorque` retain outboard reactions; optional `leftInboard`/`rightInboard`
  transfer the associated reaction to `axleFrame`. Do not connect one actuator
  to both paths or add a second support torque to the existing contact-moment
  path. Dynamic shaft transfers include carrier motion for power balance; the
  prescribed four-post rig explicitly uses static brake reactions.
- Tires live under `Chassis.Suspension` because suspension axles own wheel
  centers, tire load paths, and raw contact-patch frames.
- The public brakes subsystem lives under `Brakes`, parallel to `Chassis`,
  following VehicleInterfaces even when brake reactions attach to the chassis.
- Suspension implementations are position-independent: use the same model at
  the front or rear with the appropriate geometry and calibration. Do not copy
  an implementation just to give it a front/rear class name.
- Under `Chassis.Suspension`, keep the architecture-neutral axle contract in
  `Interfaces`, extensible partial implementations in `BaseClasses`, and
  reusable mechanisms in `Components`. Concrete axles inherit their interface
  through the shared base, rather than extending both independently.
- Wheel DOF models and `PartialWheel` live together in `Suspension.Tires.Wheels`.
  Do not reintroduce the removed suspension `Templates`, `Linkages`, or
  `TirePhysics/Templates` wrapper packages. Record package paths are unchanged.
- Chassis partial implementations live in `Chassis.BaseClasses`; concrete
  configurations stay at the chassis package level. Use model-name labels
  (`%name`) rather than explanatory captions in model graphics, with text
  extents and automatic sizing that fit the placed component. Apply this
  name-only rule throughout BobLib model graphics, not just chassis. The user
  explicitly permits `Fx, Fy` inside the contact-force actuator icon to identify
  its force axes. Package
  badges and upstream MSL/VehicleInterfaces graphics retain their standard
  appearance; do not fork standard components solely to change their labels.
- Expose the sprung frame model independently of front/rear axle choices.
  `DoubleWishbone.FrameModel` is constrained by `Body.FrameBase`; place
  compliance-only modifiers on the default compliant model selection, not on
  the shared frame instance, so rigid/custom frames remain substitutable.
- The chassis inherits its three-component assembly from
  `Chassis.BaseClasses.PartialChassisAssembly`; do not reintroduce a nested complete
  chassis component. Keep front axle, sprung body, and rear axle visible in the
  adapter diagram, but protected as implementation details of the public adapter.
- Local subsystem buses use domain-owned typed connectors under `Interfaces`,
  extending the corresponding VehicleInterfaces bus. Axles use
  `Suspension.Interfaces.AxleBus`; `Chassis.BaseClasses.PartialChassis` maps axle
  measurements into the flat chassis fields. Add each new signal to its owning
  typed bus definition before connecting it; verify typed connections with the Modelica compiler. Keep one publisher per signal.
  All BobLib controlBus ports use Utilities.Interfaces.ControlBus, including
  domain interface bases and replaceable subsystem constraints. Its subsystem
  members use BobLib-owned bus contracts; omit unused subsystem buses.
  VI ports are non-replaceable: adapt the VI interface locally with attribution,
  never redeclare an inherited non-replaceable port or modify upstream files.
  Do not maintain detached bus dictionaries under UsersGuide or modify VI files.
- Publish the standard `chassisBus.longitudinalVelocity` as signed forward
  velocity. BobLib's `vehicleSpeed` is a separate speed magnitude, not an alias.
- Brake-placement support must distinguish outboard wheel brakes, inboard
  halfshaft brakes, and a single differential-input brake. The four-post rig
  must account for the physical reaction path; the central-brake case must
  account for the differential and gearing rather than assume independent
  wheel torques. Keep this physics change separate from behavior-preserving
  package consolidation.
- Suspension axle and tire models should expose and connect raw contact-patch
  frames. MultiBody contact mechanics that close those frames to ground or
  chassis-level fixtures should not live under `Tires`; place reusable
  MultiBody contact/fixture utilities under `Utilities.Mechanics.MultiBody`.
- If a MultiBody helper is not naturally owned and used by the package exactly
  one level above it, prefer `Utilities.Mechanics.MultiBody` over burying it
  inside a deeper physics package. This includes reusable frame fixtures,
  rig actuators, and contact mechanics.
- General mechanics calculations that operate on records, arrays, or scalar
  quantities, such as mass-property combination or inertia translation, belong
  under `Utilities.Mechanics.Functions`. Frame-based helpers belong under
  `Utilities.Mechanics.MultiBody`.
- First-level packages under `Records.VehicleRecord` should mirror the public
  vehicle subsystem packages. Complete records under `Records.VehicleDefn`
  aggregate domain-owned records such as `pBattery`, `pVCU`, `pInverter`,
  `pMotor`, and `pDriveline`; avoid reintroducing a monolithic powertrain
  record as a second access path.
- Experiment implementations live in `Experiments.Standards.BaseClasses`.
  Share plant declarations and wiring in `PartialVehicle`; keep maneuver policy
  and FMI commands in their wrappers. Do not restore a production model for
  every front/rear combination; that matrix belongs to `BobLibTest.Experiments.Standards.Configurations`.
- `PartialChassis` and `PartialFourPostRig` depend on `Suspension.Interfaces.AxleBase`.
  Geometry, tire-state initialization, and wishbone-specific kinematics belong
  in the double-wishbone specializations. Axle measurements arrive through the bus.
- Keep tests in the sibling `Tests/BobLibTest` library. Production
  packages should not contain regression or component test models. Mirror each
  production model/block as `BobLibTest.<same package>.Test<ClassName>` and add
  it to `Tests/model_coverage.json`. Keep supplemental variants beside their
  domain. Partial/icon models receive structural checks; label concrete
  fixtures as component or integration tests. Do not silently regenerate
  initialization baselines when adding fixtures.
- Use `make modelica-coverage` for coverage checks, `make modelica-components`
  for behavioral tests, and `make modelica-profile PROFILE_DIR=<new directory>`
  for phase timings and package rollups. Rollups are fixture costs, not
  exclusive contributions to vehicle runtime.
- Use Modelica Standard Library 4.1.0 and VehicleInterfaces 2.0.2 for this
  comparison library unless the user explicitly changes the target versions.
- The shared `bobdyn.png` logo should remain byte-identical across BobDyn
  repositories. Put theme/background treatment in Modelica annotations, not by
  editing the PNG asset.
- Use the existing hand-drawn axle and lower-level diagrams as the styling
  reference. Inserted component symbols must never overlap. Route connections
  to avoid overlapping segments and unnecessary crossings; check inherited
  components as well as locally declared ones in the rendered diagram.
- Preserve native component icon aspect ratios unless a specific exception is
  necessary and documented. VI/MSL bus canvases are square; use square placement
  extents (including iconTransformation), retaining rotation as needed. Their
  native artwork already has the intended wide proportions.
- Prefer landscape icons and diagrams over tall, narrow blocks. Distribute
  numerous inputs across the top and sides by role, and update every caller's
  placement and wire endpoints when changing a shared icon's connector layout.
- Validate changed diagram layouts in OMEdit. Annotation coordinates alone do
  not establish that symbols, labels, and connection paths are legible.
- Apply these same visual rules to `Tests/BobLibTest`, including supplemental
  fixtures and inherited diagrams. Give every inserted component a placement
  and every connection an explicit orthogonal route. The Python diagram checks
  catch missing routes, diagonal segments, and overlapping local component
  rectangles (including nested classes); they do not replace rendered review
  of labels, shared bus junctions, or inherited symbols and wires.
- Place external connector centers on the diagram boundary, following MSL and
  VehicleInterfaces. Fit the internal layout to that boundary rather than
  enlarging the canvas around inherited ports; keep icon placements independent.

## OMEdit GUI Automation Notes

- In OMEdit, loaded libraries and the class tree live in the left `Libraries`
  pane. When that pane has focus, `Up` and `Down` traverse the library/class
  selection; do not confuse this with menu traversal.
- The library explorer is a tree view. Select it first, then use `Right` to
  expand the selected package, `Left` to collapse or move back toward the
  parent, `Up`/`Down` to move through visible rows, and `Enter` to open the
  selected class/model in the viewer. Double-clicking the row is the mouse
  equivalent of `Enter`.
- From a reset top-level library tree, the primary simulation entry point lives
  at `BobLib > Experiments > Standards > VehicleSim`; the
  four-post entry point is the sibling `FourPostSim`. Expand packages in the
  left tree rather than searching for these through the top menu bar.
- After a model is opened with `Enter`, OMEdit switches to the model viewer.
  At that point the model-specific toolbar buttons appear, including check,
  instantiate, simulation setup, and simulate controls. The simulation setup
  dialog can be opened from the toolbar or from `Simulation > Simulation Setup`.
  If the toolbar is hidden, use `View > Toolbars > Simulation Toolbar`.
- For a clean OMEdit package-load workflow, start from an OMEdit instance with
  no BobLib package loaded. Open the File menu with `Alt+F`, use arrow-key
  navigation to highlight `Open Model/Library File(s)`, and press `Enter`.
  Do not document this workflow as `Ctrl+O`; OMEdit may show that accelerator
  in the menu, but the intended capture path is menu traversal.
- For Hyprland OMEdit work, use the workspace the user assigns for that pass
  before launching or interacting with OMEdit. Earlier capture work used
  workspace 5; the later clean BobDocs pass used workspace 2 after the user
  moved VS Code to workspace 1. Scope screenshots to the OMEdit window geometry
  from `hyprctl clients -j`; capture with `grim -g "x,y widthxheight" ...`
  rather than grabbing the whole desktop. Return to the user's requested
  workspace when OMEdit work is complete.
- OMEdit window addresses can change during GUI work. Refresh the address with
  `hyprctl clients -j` before sending shortcuts or capturing a final state.
- To put keyboard focus into the `Libraries` tree from Hyprland, move the
  cursor to the desired visible row and send a left-click before using arrow
  keys:
  `hyprctl dispatch movecursor X Y`, then
  `hyprctl dispatch sendshortcut ,mouse:272,address:$ADDR`. Verify the selected
  row visually or with AT-SPI before continuing. Top-level visible rows are
  spaced by about 32 px, but the absolute row coordinate depends on the current
  workspace, window offset, panel layout, and whether any package above it is
  expanded. Verify the highlight before pressing `Right` or `Enter`.
- GI AT-SPI geometry can be useful for identifying controls, but the extents
  reported by Qt may be relative to the OMEdit window rather than raw Hyprland
  screen coordinates. If using `hyprctl dispatch movecursor`, add the window
  offset from `hyprctl clients -j` or use a visual screenshot check.
- Navigate by visible class names rather than saved arrow-key counts: expand
  `BobLib > Experiments > Standards`, then select `VehicleSim` and press
  `Enter`. `Examples` and `BaseClasses` precede the entry points, which are ordered
  `FourPostSim`, `VehicleSim`, `VehicleFMI`.
- Once `VehicleSim` is open, the model toolbar exposes buttons named
  `Check Model`, `Instantiate Model`, `Simulation Setup`, and `Simulate`.
  With `QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1`, GI AT-SPI can activate the toolbar
  button directly: find the button named `Simulation Setup`, get its action
  interface, and call `Atspi.Action.do_action(action, 0)`. This opened the
  Simulation Setup dialog reliably during the clean BobDocs capture pass.
- Prefer Hyprland's native shortcut dispatcher for menus:
  `hyprctl dispatch sendshortcut ALT,F,address:$ADDR` opens `File`.
  `hyprctl dispatch sendshortcut ALT,M,address:$ADDR` opens `Simulation`;
  `ALT,S` opens `SSP`, not `Simulation`.
- Once a menu is open, use `sendshortcut` for the relevant navigation key and
  verify with a screenshot before documenting the step. If the `Libraries`
  pane is still focused, arrow keys will move the library selection instead.
- Launching OMEdit with `QT_LINUX_ACCESSIBILITY_ALWAYS_ON=1` makes the UI
  visible to AT-SPI, which is useful for reading visible row names, selection,
  focus, and expanded/collapsed state. On this Hyprland/Qt setup, AT-SPI
  synthetic mouse/keyboard events were unreliable for opening menus and
  selecting rows, so use them for inspection rather than primary control.
- AT-SPI `select_child` on the `Libraries` tree can change selection without
  moving keyboard focus. If focus remains on a different row, arrow keys act on
  the focused row, not the selected row. Treat the visible/focused explorer row
  as the source of truth before pressing `Right`, `Left`, or `Enter`.
- Close any open menu with `Escape` before attempting explorer navigation or
  row selection. Menu overlays intercept clicks and make coordinate tests look
  wrong.

## Replacement Contracts

- Mounts belong inside their owning subsystem models. Preserve the distinction
  between the chassis attachment frame and the mounted housing/shaft frame;
  moving a mount must not move or bypass its reaction load path.
- Place wire junctions in open diagram space, clear of component icon edges.
  Only an actual connector may terminate a wire on an icon edge. Merge routes
  to the same connector before approaching it; do not merge distinct bus fields.

- Give replaceable slots explicit `constrainedby` contracts covering every member
  used by the parent. Reuse VehicleInterfaces contracts where sufficient.
- Bind shared assembly requirements and calibration on the constraining type
  so they survive redeclaration; keep default-specific tuning on the default.
- Use `redeclare replaceable` when derived selections must remain replaceable.

- The averaged electric-drive pair uses `electricMotorControlBus.availablePower`
  (signed watts): inverter publishes its limited allocation and supplies output
  voltage; motor owns the current law. Preserve independent electrical balance
  and the shared return path. Test each adapter independently as well as together.
