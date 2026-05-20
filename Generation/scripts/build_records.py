#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

from build_common import (
    DEFAULT_ALIASES,
    DEFAULT_IMPORTS,
    ModelicaFileBuilder,
    boblib_vehicledefn_dir,
    load_yaml,
    output_record_package,
    parameter_sections,
    prune_vehicle_records,
    repo_root,
    render_parameter,
    record_name_from_yaml,
    vehicle_yaml_path,
    write_text_file,
)


def render_record(data: dict[str, object], yaml_path: Path) -> str:
    record_package = output_record_package(data)  # type: ignore[arg-type]
    record_name = record_name_from_yaml(data, yaml_path)  # type: ignore[arg-type]
    builder = ModelicaFileBuilder(
        within=record_package,
        kind="record",
        name=record_name,
        end_name=record_name,
    )
    for import_path in DEFAULT_IMPORTS:
        builder.add_import(import_path)
    for alias, import_path in DEFAULT_ALIASES.items():
        builder.add_alias(alias, import_path)

    for name, modelica_type, fields in parameter_sections(data):  # type: ignore[arg-type]
        builder.add_block(render_parameter(name, modelica_type, fields), indent_spaces=2)
        builder.add_line()

    return builder.render()


def render_four_post_eval_record() -> str:
    source_path = (
        repo_root()
        / "_0_Utils"
        / "external"
        / "BobLib"
        / "BobLib"
        / "Resources"
        / "StandardRecord"
        / "FourPostEvalRecord.mo"
    )
    text = source_path.read_text(encoding="utf-8")
    return text


def build_vehicle_record(*, source_yaml: Path | None = None, overwrite: bool = True) -> Path:
    source_yaml = source_yaml or vehicle_yaml_path()
    data = load_yaml(source_yaml)
    record_path = boblib_vehicledefn_dir(data) / f"{record_name_from_yaml(data, source_yaml)}.mo"  # type: ignore[arg-type]
    record_path.parent.mkdir(parents=True, exist_ok=True)
    write_text_file(record_path, render_record(data, source_yaml), overwrite)  # type: ignore[arg-type]
    prune_vehicle_records(data, source_yaml)
    return record_path


def build_four_post_eval_record(*, source_yaml: Path | None = None, overwrite: bool = True) -> Path:
    source_yaml = source_yaml or vehicle_yaml_path()
    record_path = (
        repo_root()
        / "_0_Utils"
        / "external"
        / "BobLib"
        / "BobLib"
        / "Resources"
        / "StandardRecord"
        / "FourPostEvalRecord.mo"
    )
    record_path.parent.mkdir(parents=True, exist_ok=True)
    write_text_file(record_path, render_four_post_eval_record(), overwrite)
    # Keep the record package minimal and ensure the package order is coherent.
    data = load_yaml(source_yaml)
    prune_vehicle_records(data, source_yaml)
    return record_path


def main() -> None:
    source_yaml = vehicle_yaml_path()
    record_path = build_vehicle_record(source_yaml=source_yaml, overwrite=True)
    four_post_record_path = build_four_post_eval_record(overwrite=True)

    print(f"Generated BobLib record files from {source_yaml}")
    print(f"  Vehicle record:      {record_path}")
    print(f"  FourPostEval record: {four_post_record_path}")


if __name__ == "__main__":
    main()
