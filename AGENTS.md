# Agent Notes

These rules capture the current BobLib/BobLib architecture so
future agents preserve the package boundaries.

## BobLib Architecture

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
- Tires live under `Chassis.Suspension` because suspension axles own wheel
  centers, tire load paths, and raw contact-patch frames.
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
- Keep tests in the sibling `Tests` library. Production
  packages should not contain regression or component test models.
- Use Modelica Standard Library 4.1.0 and VehicleInterfaces 2.0.2 for this
  comparison library unless the user explicitly changes the target versions.
- The shared `bobdyn.png` logo should remain byte-identical across BobDyn
  repositories. Put theme/background treatment in Modelica annotations, not by
  editing the PNG asset.

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
- Verified complete traversal from a reset top-level tree to `VehicleSim`,
  using the current `package.order`: click/select `BobLib`,
  send `Right`, then send `Down`, `Down`, `Right`, `Down`, `Right`,
  `Down`, `Down`, and finally `Enter`. That expands
  `BobLib > Experiments > Standards` and opens `VehicleSim`.
  `FourPostSim` is one `Down` below `VehicleSim`.
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
