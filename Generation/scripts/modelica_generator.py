#!/usr/bin/env python3
"""
Shared helpers for generating Modelica source files.

The goal of this module is to keep Modelica rendering logic in one place so the
Python build scripts can assemble .mo files without scattering ad-hoc string
formatting across the repo.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


def indent(text: str, spaces: int = 2) -> str:
    pad = " " * spaces
    return "\n".join(pad + line if line else line for line in text.splitlines())


def modelica_number(value: int | float) -> str:
    return repr(value)


def modelica_value(value: Any) -> str:
    if isinstance(value, dict):
        if "Value" in value:
            return modelica_value(value["Value"])
        if "value" in value:
            return modelica_value(value["value"])
        if "expr" in value:
            return str(value["expr"])
        if "Expression" in value:
            return str(value["Expression"])
        if "table" in value:
            return modelica_table(value["table"])
        if "constructor" in value:
            return render_constructor(str(value["constructor"]), value.get("fields", {}))
        raise TypeError(f"Unsupported Modelica expression dict: {value!r}")
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return modelica_number(value)
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        if value and all(isinstance(row, list) for row in value):
            rows = [", ".join(modelica_value(item) for item in row) for row in value]
            return "{" + ", ".join("{" + row + "}" for row in rows) + "}"
        return "{" + ", ".join(modelica_value(item) for item in value) + "}"
    raise TypeError(f"Unsupported value for Modelica rendering: {value!r}")


def modelica_table(rows: list[list[Any]]) -> str:
    return "[" + "; ".join(", ".join(modelica_value(v) for v in row) for row in rows) + "]"


def constructor(name: str, fields: dict[str, Any]) -> dict[str, Any]:
    return {"constructor": name, "fields": fields}


def render_constructor(name: str, fields: dict[str, Any]) -> str:
    if not fields:
        return f"{name}()"
    body = ", ".join(f"{key} = {render_field_value(value)}" for key, value in fields.items())
    return f"{name}({body})"


def render_field_value(value: Any) -> str:
    if isinstance(value, dict):
        if "table" in value:
            return modelica_table(value["table"])
        if "constructor" in value:
            return render_constructor(str(value["constructor"]), value.get("fields", {}))
    return modelica_value(value)


def render_parameter(name: str, modelica_type: str, fields: dict[str, Any]) -> str:
    binding = fields.get("_binding")
    if binding is not None:
        if len(fields) != 1:
            raise ValueError(f"Binding-only parameter {name} cannot mix with field assignments: {fields!r}")
        return f"parameter {modelica_type} {name} = {binding};"
    if not fields:
        return f"parameter {modelica_type} {name};"
    assignments = [f"    {key} = {render_field_value(value)}" for key, value in fields.items()]
    return f"parameter {modelica_type} {name}(\n" + ",\n".join(assignments) + ");"


def replace_tokens(text: str, replacements: Mapping[str, str]) -> str:
    out = text
    for needle, replacement in replacements.items():
        out = out.replace(needle, replacement)
    return out


@dataclass
class ModelicaFileBuilder:
    within: str
    kind: str
    name: str
    description: str | None = None
    imports: list[str] = field(default_factory=list)
    aliases: dict[str, str] = field(default_factory=dict)
    body: list[str] = field(default_factory=list)
    end_name: str | None = None

    def add_import(self, import_path: str) -> None:
        if import_path not in self.imports:
            self.imports.append(import_path)

    def add_alias(self, alias: str, import_path: str) -> None:
        self.aliases[alias] = import_path

    def add_line(self, line: str = "") -> None:
        self.body.append(line)

    def add_block(self, text: str, indent_spaces: int = 0) -> None:
        block = indent(text, indent_spaces) if indent_spaces else text
        self.body.extend(block.splitlines())

    def extend_body(self, lines: Iterable[str]) -> None:
        self.body.extend(lines)

    def render(self) -> str:
        lines: list[str] = [f"within {self.within};", ""]
        head = f"{self.kind} {self.name}"
        if self.description:
            head += f' "{self.description}"'
        lines.extend([head, ""])
        for import_path in self.imports:
            lines.append(f"  import {import_path};")
        if self.aliases:
            if self.imports:
                lines.append("")
            for alias, import_path in self.aliases.items():
                lines.append(f"  import {alias} = {import_path};")
        if self.body:
            if self.imports or self.aliases:
                lines.append("")
            lines.extend(self.body)
        lines.append("")
        lines.append(f"end {self.end_name or self.name};")
        return "\n".join(lines).rstrip() + "\n"


def render_modelica_file(
    *,
    within: str,
    kind: str,
    name: str,
    description: str | None = None,
    imports: Iterable[str] = (),
    aliases: Mapping[str, str] | None = None,
    body: Iterable[str] = (),
    end_name: str | None = None,
) -> str:
    builder = ModelicaFileBuilder(within=within, kind=kind, name=name, description=description, end_name=end_name)
    for import_path in imports:
        builder.add_import(import_path)
    for alias, import_path in (aliases or {}).items():
        builder.add_alias(alias, import_path)
    builder.extend_body(body)
    return builder.render()
