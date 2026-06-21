from __future__ import annotations

from pathlib import Path

import modelica_linter


def test_format_modelica_text_normalizes_spacing_and_joined_sections() -> None:
    source = (
        "model Example\n"
        '  parameter SI.Length x0=0 "Initial" annotation(Dialog(group="Initialization"));\n'
        "  Modelica.Blocks.Sources.Constant c(k=1, y(start=0, fixed=true))  annotation(\n"
        "    Placement(transformation(origin={0,0},extent={{-10,-10},{10,10}})));equation\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert 'x0 = 0 "Initial" annotation(Dialog(group = "Initialization"));' in formatted
    assert "c(k = 1, y(start = 0, fixed = true)) annotation(" in formatted
    assert "origin = {0, 0}, extent = {{-10, -10}, {10, 10}}" in formatted
    assert "));\n\nequation\n" in formatted


def test_format_modelica_text_splits_long_component_modifier_calls() -> None:
    source = (
        "model Example\n"
        "  Modelica.Mechanics.MultiBody.Joints.Prismatic FreeX("
        "n = {1, 0, 0}, s(start = x0, fixed = true), "
        "v(start = vx0, fixed = true)) annotation(\n"
        "    Placement(transformation(extent = {{-10, -10}, {10, 10}})));\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source, max_line_length=88)

    assert (
        "  Modelica.Mechanics.MultiBody.Joints.Prismatic FreeX(\n"
        "    n = {1, 0, 0},\n"
        "    s(start = x0, fixed = true),\n"
        "    v(start = vx0, fixed = true)) annotation("
    ) in formatted


def test_format_modelica_text_does_not_emit_trailing_spaces() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  y=\n"
        "    f(x);\n"
        "  z = x; // inline comment\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  y =\n" in formatted
    assert "  z = x; // inline comment\n" in formatted
    assert all(line == line.rstrip() for line in formatted.splitlines())


def test_check_paths_reports_unformatted_modelica_file(tmp_path: Path) -> None:
    modelica_file = tmp_path / "Example.mo"
    modelica_file.write_text("model Example\n  parameter Real x=1;\nend Example;\n")

    diagnostics = modelica_linter.check_paths((tmp_path,))

    assert len(diagnostics) == 1
    assert diagnostics[0].path == modelica_file
    assert diagnostics[0].code == "MF001"


def test_format_paths_rewrites_modelica_files(tmp_path: Path) -> None:
    modelica_file = tmp_path / "Example.mo"
    modelica_file.write_text("model Example\n  parameter Real x=1;\nend Example;\n")

    changed = modelica_linter.format_paths((tmp_path,))

    assert changed == (modelica_file,)
    assert modelica_file.read_text() == "model Example\n\n  parameter Real x = 1;\nend Example;\n"


def test_format_modelica_text_preserves_control_keyword_spacing() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  when (x > 0) then\n"
        "    y = 1;\n"
        "  elsewhen (x < 0) then\n"
        "    y = -1;\n"
        "  end when;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  when (x > 0) then\n" in formatted
    assert "  elsewhen (x < 0) then\n" in formatted


def test_format_modelica_text_preserves_top_level_annotation_indent() -> None:
    source = (
        "model Example\n"
        "  annotation(\n"
        "    experiment(StartTime=0));\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  annotation(\n" in formatted
    assert "    experiment(StartTime = 0));\n" in formatted


def test_format_modelica_text_preserves_comment_indentation() -> None:
    source = (
        "model Example\n"
        "\t// Keep this tabbed comment exactly where it is\n"
        "    // Keep this spaced comment too\n"
        "  parameter Real x=1;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "\t// Keep this tabbed comment exactly where it is\n" in formatted
    assert "    // Keep this spaced comment too\n" in formatted


def test_format_modelica_text_aligns_unindented_comments_to_neighboring_code() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "// Explain the block\n"
        "// across two lines.\n"
        "  when sample(0,1) then\n"
        "    x = pre(x) + 1;\n"
        "  end when;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "\n  // Explain the block\n  // across two lines.\n  when sample(0, 1) then\n" in formatted


def test_format_modelica_text_adds_gaps_around_model_blocks_and_sections() -> None:
    source = (
        "model Example\n"
        "  parameter Real x=1;\n"
        "  Real y;\n"
        "  Modelica.Blocks.Sources.Constant c(k=1) annotation(\n"
        "    Placement(transformation(extent={{-10,-10},{10,10}})));\n"
        "  Modelica.Blocks.Sources.Constant d(k=2) annotation(\n"
        "    Placement(transformation(extent={{-10,-10},{10,10}})));\n"
        "protected\n"
        "  Real z;\n"
        "equation\n"
        "  y = c.y;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "model Example\n\n  parameter Real x = 1;" in formatted
    assert "  Real y;\n\n  Modelica.Blocks.Sources.Constant c" in formatted
    assert "  Modelica.Blocks.Sources.Constant c(k = 1) annotation(\n    Placement(transformation(extent = {{-10, -10}, {10, 10}})));\n\n  Modelica.Blocks.Sources.Constant d" in formatted
    assert "  Placement(transformation(extent = {{-10, -10}, {10, 10}})));\n\nprotected\n" in formatted
    assert "  Real z;\n\nequation\n" in formatted


def test_format_modelica_text_keeps_comments_attached_to_following_model_block() -> None:
    source = (
        "model Example\n"
        "  parameter Real x=1;\n"
        "  // Source block\n"
        "  Modelica.Blocks.Sources.Constant c(k=1);\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  parameter Real x = 1;\n\n  // Source block\n  Modelica.Blocks" in formatted


def test_format_modelica_text_adds_gap_before_control_blocks() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  y = x;\n"
        "  if x > 0 then\n"
        "    y = 1;\n"
        "  end if;\n"
        "  z = y;\n"
        "  when sample(0, 1) then\n"
        "    if y > 0 then\n"
        "      z = y;\n"
        "    end if;\n"
        "    y = pre(y) + 1;\n"
        "  end when;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  y = x;\n\n  if x > 0 then\n" in formatted
    assert "  z = y;\n\n  when sample(0, 1) then\n" in formatted
    assert "  when sample(0, 1) then\n    if y > 0 then\n" in formatted


def test_format_modelica_text_splits_jumbled_control_blocks() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  y = x;if x > 0 then\n"
        "    y = 1;end if;\n"
        "  when sample(0,1) then\n"
        "    z = y;elsewhen sample(0,2) then\n"
        "    z = 0;end when;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  y = x;\n\n  if x > 0 then\n" in formatted
    assert "    y = 1;\n  end if;\n" in formatted
    assert "    z = y;\n  elsewhen sample(0, 2) then\n" in formatted
    assert "    z = 0;\n  end when;\n" in formatted


def test_format_modelica_text_splits_inline_conditional_expressions() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  y = if a then b elseif c then d\n"
        "   elseif e then f else g;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  y = if a then b\n    elseif c then d\n" in formatted
    assert "    elseif e then f\n    else g;\n" in formatted


def test_format_modelica_text_keeps_short_inline_conditional_expressions() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  direction = if noEvent(target >= 0) then 1 else -1;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "  direction = if noEvent(target >= 0) then 1 else -1;\n" in formatted


def test_format_modelica_text_preserves_block_elseif_after_wrapped_branch() -> None:
    source = (
        "model Example\n"
        "equation\n"
        "  y =\n"
        "\n"
        "    if enabled then\n"
        "      noEvent(min(\n"
        "        limit,\n"
        "        max(0, demand)))\n"
        "    elseif fallback then\n"
        "      0\n"
        "    else\n"
        "      demand;\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "\n    elseif fallback then\n" in formatted
    assert "\n          elseif fallback then\n" not in formatted


def test_format_modelica_text_preserves_multiline_documentation_content() -> None:
    source = (
        "model Example\n"
        "  annotation(\n"
        "    Documentation(info=\"<html>\n"
        "<p><code>useMode=3</code>, a=b, and x,y should stay literal.</p>\n"
        "</html>\"));\n"
        "end Example;\n"
    )

    formatted = modelica_linter.format_modelica_text(source)

    assert "<code>useMode=3</code>, a=b, and x,y should stay literal." in formatted
