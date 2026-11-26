# Native binary wrappers

This directory is the home for adapters around OpenModelica-generated native
binaries and equation libraries. Modelica physics belongs in `BobLib/`; build and
export entry points remain in the parent `Tools/` directory.

## Layout and ownership

Keep runtime-specific C bridges and their Python launch/load adapters together
here. Put regression tests and benchmark drivers in `Tests/`. Generated shared
libraries, manifests, logs and simulation output belong under the ignored root
`build/` directory or an explicitly selected external output directory.

A native adapter must document its supported compiler/runtime ABI, process and
thread lifetime, artifact validation, failure handling, and opt-in activation.
The adapter's tests and build entry point should land with its implementation.

## Available entry points

Run from the repository root:

```sh
python3 Tools/build_native.py --help
python3 Tools/verify_fmu.py --help
```

The [export guide](../README.md) covers fixed-architecture native executables,
editable calibration and comparison with FMUs. Keep an executable with the
runtime files listed in its manifest.

The solver-cache and direct-DAE adapters are ongoing development work. Their
implementations are not part of this housekeeping change, and this directory
does not enable an accelerator in BobSim or alter the installed OpenModelica runtime.
