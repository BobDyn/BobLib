from __future__ import annotations

import argparse
import difflib
import re
import sys
from dataclasses import dataclass
from pathlib import Path


DEFAULT_MAX_LINE_LENGTH = 120
SECTION_KEYWORDS = (
    "equation",
    "algorithm",
    "protected",
    "public",
    "initial equation",
    "initial algorithm",
)
JOINED_CONTROL_START_KEYWORDS = (
    "if ",
    "if(",
    "when ",
    "when(",
)
JOINED_CONTROL_CONTINUATION_KEYWORDS = (
    "else",
    "elseif ",
    "elsewhen ",
    "end if",
    "end when",
)
SKIP_LONG_CALL_MARKERS = (
    "Documentation(",
    "<html>",
    "Bitmap(",
    "Line(points",
    "Polygon(points",
    "Rectangle(",
    "Ellipse(",
    "Text(",
)
CONTROL_WORDS_BEFORE_PAREN = {
    "and",
    "elsewhen",
    "for",
    "if",
    "in",
    "not",
    "or",
    "then",
    "when",
    "while",
}
DECLARATION_QUALIFIERS = {
    "constant",
    "discrete",
    "final",
    "flow",
    "input",
    "inner",
    "outer",
    "output",
    "parameter",
    "replaceable",
    "stream",
}
SCALAR_TYPE_WORDS = {
    "Boolean",
    "Integer",
    "Real",
    "String",
}
SCALAR_TYPE_PREFIXES = (
    "SI.",
    "Modelica.Units.SI.",
    "Modelica.SIunits.",
)


@dataclass(frozen=True)
class Diagnostic:
    path: Path
    line: int
    code: str
    message: str


def iter_modelica_files(paths: tuple[Path, ...]) -> tuple[Path, ...]:
    files: list[Path] = []
    for path in paths:
        if path.is_file() and path.suffix == ".mo":
            files.append(path)
        elif path.is_dir():
            files.extend(sorted(path.rglob("*.mo")))
    return tuple(dict.fromkeys(files))


def format_modelica_text(text: str, *, max_line_length: int = DEFAULT_MAX_LINE_LENGTH) -> str:
    newline = "\r\n" if "\r\n" in text else "\n"
    formatted = text.replace("\r\n", "\n").replace("\r", "\n")
    formatted = _split_joined_sections(formatted)

    lines: list[str] = []
    in_multiline_string = False
    for raw_line in formatted.split("\n"):
        if in_multiline_string:
            lines.append(raw_line.rstrip())
            in_multiline_string = _string_is_open_after_line(raw_line, True)
            continue

        if _is_comment_only_line(raw_line):
            lines.append(raw_line.rstrip())
            in_multiline_string = _string_is_open_after_line(raw_line, False)
            continue

        line = raw_line.replace("\t", "  ").rstrip()
        line = _normalize_code_spacing(line)
        for inline_line in _split_inline_conditional_expression(
            line,
            max_line_length=max_line_length,
        ):
            lines.extend(_split_long_call_line(inline_line, max_line_length=max_line_length))
        in_multiline_string = _string_is_open_after_line(line, False)

    lines = _insert_layout_blank_lines(lines)
    formatted = "\n".join(lines)
    if text.endswith(("\n", "\r\n")) and not formatted.endswith("\n"):
        formatted += "\n"
    return formatted.replace("\n", newline)


def _is_comment_only_line(line: str) -> bool:
    stripped = line.lstrip()
    return stripped.startswith("//") or stripped.startswith("/*") or stripped.startswith("*")


def _align_comment_indent(line: str, lines: list[str], index: int) -> str:
    if _line_indent(line):
        return line
    return f"{_neighbor_code_indent(lines, index)}{line.lstrip()}"


def _neighbor_code_indent(lines: list[str], index: int) -> str:
    for line in lines[index + 1 :]:
        if line.strip() and not _is_comment_only_line(line):
            return _line_indent(line)
    for line in reversed(lines[:index]):
        if line.strip() and not _is_comment_only_line(line):
            return _line_indent(line)
    return ""


def _line_indent(line: str) -> str:
    match = re.match(r"[ \t]*", line)
    return match.group(0) if match else ""


def _string_is_open_after_line(line: str, initial_state: bool) -> bool:
    in_string = initial_state
    in_line_comment = False
    index = 0
    while index < len(line):
        char = line[index]
        next_char = line[index + 1] if index + 1 < len(line) else ""

        if in_line_comment:
            break

        if in_string:
            if char == "\\" and next_char:
                index += 2
                continue
            if char == '"':
                in_string = False
            index += 1
            continue

        if char == "/" and next_char == "/":
            in_line_comment = True
            index += 2
            continue

        if char == '"':
            in_string = True

        index += 1

    return in_string


def _insert_layout_blank_lines(lines: list[str]) -> list[str]:
    output: list[str] = []
    section = "declaration"
    previous_kind: str | None = None
    in_scalar_declaration = False
    in_component_declaration = False

    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped:
            if output and output[-1] != "":
                output.append("")
            continue

        is_comment = _is_comment_only_line(line)
        if is_comment:
            line = _align_comment_indent(line, lines, index)
            stripped = line.strip()
        else:
            line = _normalize_inline_conditional_continuation_indent(line, output)
            stripped = line.strip()

        if is_comment and previous_kind != "comment" and output and output[-1] != "":
            output.append("")

        starts_control_block = _is_control_block_start(stripped)
        if (
            starts_control_block
            and previous_kind not in {None, "section", "comment", "control"}
            and output
            and output[-1] != ""
        ):
            output.append("")

        if _is_section_header(stripped) and output and output[-1] != "":
            output.append("")

        starts_scalar = (
            section not in {"equation", "algorithm"}
            and not in_scalar_declaration
            and not in_component_declaration
            and _is_scalar_declaration_start(line)
        )
        starts_component = (
            section not in {"equation", "algorithm"}
            and not in_scalar_declaration
            and not in_component_declaration
            and not starts_scalar
            and _is_component_declaration_start(line)
        )
        if (
            starts_component
            and previous_kind in {"scalar", "component", "other"}
            and output
            and output[-1] != ""
        ):
            output.append("")

        output.append(line)

        if _is_class_header(stripped) and _next_nonblank_line(lines, index) not in {"", None}:
            output.append("")
            previous_kind = "class"
            continue

        if _is_section_header(stripped):
            section = _section_kind(stripped)
            previous_kind = "section"
            in_scalar_declaration = False
            in_component_declaration = False
            continue

        if starts_scalar:
            in_scalar_declaration = not stripped.endswith(";")
            previous_kind = "scalar"
            continue

        if in_scalar_declaration:
            if stripped.endswith(";"):
                in_scalar_declaration = False
            previous_kind = "scalar"
            continue

        if starts_component:
            in_component_declaration = not stripped.endswith(";")
            previous_kind = "component"
            continue

        if in_component_declaration:
            if stripped.endswith(";"):
                in_component_declaration = False
            previous_kind = "component"
            continue

        if starts_control_block:
            previous_kind = "control"
        elif section not in {"equation", "algorithm"} and _is_scalar_declaration_start(line):
            previous_kind = "scalar"
        elif _is_comment_only_line(line):
            previous_kind = "comment"
        else:
            previous_kind = "other"

    while len(output) > 1 and output[-1] == "" and output[-2] == "":
        output.pop()
    return output


def _next_nonblank_line(lines: list[str], index: int) -> str | None:
    for line in lines[index + 1 :]:
        if line.strip():
            return line
        if line == "":
            return ""
    return None


def _is_section_header(stripped: str) -> bool:
    return stripped in SECTION_KEYWORDS


def _is_control_block_start(stripped: str) -> bool:
    code = _split_line_comment(stripped)[0].strip()
    return (
        code.startswith(("if ", "if("))
        or code.startswith(("when ", "when("))
    ) and code.endswith("then")


def _section_kind(stripped: str) -> str:
    if stripped.endswith("algorithm"):
        return "algorithm"
    if stripped.endswith("equation"):
        return "equation"
    return "declaration"


def _is_class_header(stripped: str) -> bool:
    if stripped.startswith("end "):
        return False
    class_prefixes = (
        "block ",
        "connector ",
        "expandable connector ",
        "function ",
        "model ",
        "operator record ",
        "package ",
        "partial block ",
        "partial function ",
        "partial model ",
        "partial package ",
        "partial record ",
        "record ",
        "type ",
    )
    return stripped.startswith(class_prefixes)


def _is_component_declaration_start(line: str) -> bool:
    stripped = line.strip()
    if not stripped or _is_comment_only_line(line):
        return False
    if stripped.startswith(("connect(", "assert(", "reinit(", "terminate(")):
        return False

    annotation_index = stripped.find(" annotation(")
    search_text = stripped[:annotation_index] if annotation_index >= 0 else stripped
    open_index = search_text.find("(")
    if open_index < 0:
        return False

    prefix = search_text[:open_index].strip()
    if not prefix:
        return False
    tokens = prefix.split()
    if len(tokens) < 2:
        return False
    if tokens[-1] in {"annotation", "if", "for", "when", "while"}:
        return False
    if "=" in prefix:
        return False
    type_token = _declaration_type_token(prefix)
    if _is_scalar_type(type_token):
        return False
    return True


def _is_scalar_declaration_start(line: str) -> bool:
    stripped = line.strip()
    if not stripped or _is_comment_only_line(line):
        return False
    if stripped.startswith(("connect(", "assert(", "reinit(", "terminate(")):
        return False
    if stripped.startswith("import "):
        return True
    type_token = _declaration_type_token(_split_line_comment(stripped)[0])
    return _is_scalar_type(type_token)


def _declaration_type_token(text: str) -> str | None:
    declaration = re.split(r"\s+annotation\s*\(", text, maxsplit=1)[0].strip()
    declaration = declaration.split('"', maxsplit=1)[0].strip()
    declaration = re.split(r"\s+if\s+", declaration, maxsplit=1)[0].strip()
    declaration = declaration.replace(";", " ")
    tokens = declaration.split()
    index = 0
    while index < len(tokens) and tokens[index] in DECLARATION_QUALIFIERS:
        index += 1
    if len(tokens) - index < 2:
        return None
    return tokens[index].split("[", maxsplit=1)[0]


def _is_scalar_type(type_token: str | None) -> bool:
    if type_token is None:
        return False
    return type_token in SCALAR_TYPE_WORDS or type_token.startswith(SCALAR_TYPE_PREFIXES)


def check_paths(
    paths: tuple[Path, ...],
    *,
    max_line_length: int = DEFAULT_MAX_LINE_LENGTH,
) -> tuple[Diagnostic, ...]:
    diagnostics: list[Diagnostic] = []
    for path in iter_modelica_files(paths):
        source = path.read_text(encoding="utf-8")
        formatted = format_modelica_text(source, max_line_length=max_line_length)
        if formatted != source:
            diagnostics.append(
                Diagnostic(
                    path=path,
                    line=1,
                    code="MF001",
                    message="Modelica file is not in BobLib formatting style",
                )
            )
    return tuple(diagnostics)


def format_paths(
    paths: tuple[Path, ...],
    *,
    max_line_length: int = DEFAULT_MAX_LINE_LENGTH,
) -> tuple[Path, ...]:
    changed: list[Path] = []
    for path in iter_modelica_files(paths):
        source = path.read_text(encoding="utf-8")
        formatted = format_modelica_text(source, max_line_length=max_line_length)
        if formatted != source:
            path.write_text(formatted, encoding="utf-8")
            changed.append(path)
    return tuple(changed)


def diff_paths(
    paths: tuple[Path, ...],
    *,
    max_line_length: int = DEFAULT_MAX_LINE_LENGTH,
) -> str:
    chunks: list[str] = []
    for path in iter_modelica_files(paths):
        source = path.read_text(encoding="utf-8")
        formatted = format_modelica_text(source, max_line_length=max_line_length)
        if formatted == source:
            continue
        chunks.extend(
            difflib.unified_diff(
                source.splitlines(keepends=True),
                formatted.splitlines(keepends=True),
                fromfile=str(path),
                tofile=f"{path} (formatted)",
            )
        )
    return "".join(chunks)


def _split_joined_sections(text: str) -> str:
    output: list[str] = []
    i = 0
    in_string = False
    in_line_comment = False
    in_block_comment = False
    while i < len(text):
        char = text[i]
        next_char = text[i + 1] if i + 1 < len(text) else ""

        if in_line_comment:
            output.append(char)
            if char == "\n":
                in_line_comment = False
            i += 1
            continue

        if in_block_comment:
            output.append(char)
            if char == "*" and next_char == "/":
                output.append(next_char)
                i += 2
                in_block_comment = False
            else:
                i += 1
            continue

        if in_string:
            output.append(char)
            if char == "\\" and next_char:
                output.append(next_char)
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
            continue

        if char == "/" and next_char == "/":
            output.append(char)
            output.append(next_char)
            i += 2
            in_line_comment = True
            continue

        if char == "/" and next_char == "*":
            output.append(char)
            output.append(next_char)
            i += 2
            in_block_comment = True
            continue

        if char == '"':
            output.append(char)
            i += 1
            in_string = True
            continue

        if char == ";":
            output.append(char)
            j = i + 1
            while j < len(text) and text[j] in " \t":
                j += 1
            if j < len(text) and text[j] != "\n":
                remaining = text[j:]
                joined_kind = _joined_line_start_kind(remaining)
                if joined_kind is not None:
                    indent = _current_output_line_indent(output)
                    output.append("\n")
                    if joined_kind == "control_start":
                        output.append(indent)
                    elif joined_kind == "control_continuation":
                        output.append(_dedent_indent(indent))
                    i = j
                    continue
            i += 1
            continue

        output.append(char)
        i += 1

    return "".join(output)


def _joined_line_start_kind(text: str) -> str | None:
    if any(text.startswith(keyword) for keyword in SECTION_KEYWORDS):
        return "section"
    if any(text.startswith(keyword) for keyword in JOINED_CONTROL_START_KEYWORDS):
        return "control_start"
    if any(
        text.startswith(keyword)
        for keyword in JOINED_CONTROL_CONTINUATION_KEYWORDS
    ):
        return "control_continuation"
    return None


def _current_output_line_indent(output: list[str]) -> str:
    line_start = len(output) - 1
    while line_start >= 0 and output[line_start] != "\n":
        line_start -= 1
    current_line = "".join(output[line_start + 1 :])
    match = re.match(r"[ \t]*", current_line)
    return match.group(0) if match else ""


def _dedent_indent(indent: str) -> str:
    if indent.endswith("  "):
        return indent[:-2]
    if indent.endswith("\t"):
        return indent[:-1]
    return ""


def _normalize_code_spacing(line: str) -> str:
    code, comment = _split_line_comment(line)
    code = _remove_space_before_call_paren(code)
    code = _space_commas_and_equals(code)
    code = re.sub(r"(?<=\S)\s{2,}annotation\(", " annotation(", code)
    code = code.rstrip()
    if comment and code:
        return f"{code} {comment}"
    return code + comment


def _split_inline_conditional_expression(
    line: str,
    *,
    max_line_length: int,
) -> list[str]:
    stripped = line.lstrip()
    if stripped.startswith(("if ", "if(", "else ", "when ", "when(")):
        return [line]
    if not stripped.startswith("elseif ") and " if " not in line and " = if " not in line:
        return [line]
    if (
        not stripped.startswith("elseif ")
        and " elseif " not in line
        and len(line) <= max_line_length
    ):
        return [line]

    keywords = (" else ",) if stripped.startswith("elseif ") else (" elseif ", " else ")
    positions = _top_level_keyword_positions(line, keywords)
    if not positions:
        return [line]

    indent = _line_indent(line)
    continuation_indent = indent if stripped.startswith("elseif ") else indent + "  "
    split_lines: list[str] = []
    start = 0
    for position in positions:
        split_lines.append(line[start:position].rstrip())
        start = position + 1
    split_lines.append(line[start:].strip())

    return [split_lines[0], *(f"{continuation_indent}{part}" for part in split_lines[1:])]


def _top_level_keyword_positions(text: str, keywords: tuple[str, ...]) -> list[int]:
    positions: list[int] = []
    paren_depth = 0
    brace_depth = 0
    bracket_depth = 0
    in_string = False
    index = 0
    while index < len(text):
        char = text[index]
        next_char = text[index + 1] if index + 1 < len(text) else ""

        if in_string:
            if char == "\\" and next_char:
                index += 2
                continue
            if char == '"':
                in_string = False
            index += 1
            continue

        if char == '"':
            in_string = True
        elif char == "(":
            paren_depth += 1
        elif char == ")":
            paren_depth -= 1
        elif char == "{":
            brace_depth += 1
        elif char == "}":
            brace_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif paren_depth == 0 and brace_depth == 0 and bracket_depth == 0:
            for keyword in keywords:
                if text.startswith(keyword, index):
                    positions.append(index)
                    index += len(keyword) - 1
                    break

        index += 1

    return positions


def _normalize_inline_conditional_continuation_indent(line: str, output: list[str]) -> str:
    stripped = line.strip()
    if not stripped.startswith(("elseif ", "else ")):
        return line

    previous_line = _previous_output_code_line(output)
    if previous_line is None or previous_line.strip().endswith((";", "then")):
        return line
    if stripped.startswith("elseif ") and _has_open_block_if_at_indent(output, _line_indent(line)):
        return line

    if previous_line.strip().startswith(("elseif ", "else ")):
        indent = _line_indent(previous_line)
    else:
        indent = _line_indent(previous_line) + "  "
    return f"{indent}{stripped}"


def _has_open_block_if_at_indent(output: list[str], indent: str) -> bool:
    for previous_line in reversed(output):
        stripped = previous_line.strip()
        if not stripped or _is_comment_only_line(previous_line):
            continue
        previous_indent = _line_indent(previous_line)
        if len(previous_indent) < len(indent):
            return False
        if previous_indent == indent and stripped.startswith(("if ", "if(")) and stripped.endswith("then"):
            return True
    return False


def _previous_output_code_line(output: list[str]) -> str | None:
    for line in reversed(output):
        if line.strip() and not _is_comment_only_line(line):
            return line
    return None


def _split_line_comment(line: str) -> tuple[str, str]:
    in_string = False
    i = 0
    while i < len(line) - 1:
        char = line[i]
        next_char = line[i + 1]
        if in_string:
            if char == "\\":
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
            continue
        if char == '"':
            in_string = True
            i += 1
            continue
        if char == "/" and next_char == "/":
            return line[:i], line[i:]
        i += 1
    return line, ""


def _remove_space_before_call_paren(code: str) -> str:
    output: list[str] = []
    in_string = False
    i = 0
    while i < len(code):
        char = code[i]
        if in_string:
            output.append(char)
            if char == "\\" and i + 1 < len(code):
                output.append(code[i + 1])
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            output.append(char)
            in_string = True
            i += 1
            continue

        if char == " " and i + 1 < len(code) and code[i + 1] == "(":
            previous = output[-1] if output else ""
            previous_word = _previous_word("".join(output))
            if (
                previous.isalnum() or previous == "_"
            ) and previous_word not in CONTROL_WORDS_BEFORE_PAREN:
                i += 1
                continue

        output.append(char)
        i += 1
    return "".join(output)


def _previous_word(text: str) -> str:
    match = re.search(r"([A-Za-z_]\w*)$", text)
    return match.group(1) if match else ""


def _space_commas_and_equals(code: str) -> str:
    output: list[str] = []
    in_string = False
    i = 0
    while i < len(code):
        char = code[i]
        next_char = code[i + 1] if i + 1 < len(code) else ""
        previous = code[i - 1] if i > 0 else ""

        if in_string:
            output.append(char)
            if char == "\\" and next_char:
                output.append(next_char)
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            output.append(char)
            in_string = True
            i += 1
            continue

        if char == ",":
            output.append(char)
            if next_char and not next_char.isspace() and next_char not in ")}]":
                output.append(" ")
            i += 1
            continue

        if char == "=" and previous not in "<>=:" and next_char != "=":
            while output and output[-1] == " ":
                output.pop()
            output.append(" = ")
            i += 1
            while i < len(code) and code[i] == " ":
                i += 1
            continue

        output.append(char)
        i += 1

    return "".join(output)


def _split_long_call_line(line: str, *, max_line_length: int) -> list[str]:
    if len(line) <= max_line_length:
        return [line]
    if any(marker in line for marker in SKIP_LONG_CALL_MARKERS):
        return [line]

    annotation_index = line.find(" annotation(")
    search_limit = annotation_index if annotation_index >= 0 else len(line)
    open_index = line.find("(")
    if open_index < 0 or open_index >= search_limit:
        return [line]

    close_index = _matching_paren_index(line, open_index)
    if close_index is None or close_index >= search_limit:
        return [line]

    inner = line[open_index + 1 : close_index]
    arguments = _split_top_level_commas(inner)
    if len(arguments) < 2:
        return [line]

    indent = line[: len(line) - len(line.lstrip())]
    prefix = line[:open_index].rstrip()
    suffix = line[close_index + 1 :]
    argument_indent = indent + "  "
    split_lines = [f"{prefix}("]
    for argument in arguments[:-1]:
        split_lines.append(f"{argument_indent}{argument.strip()},")
    split_lines.append(f"{argument_indent}{arguments[-1].strip()}){suffix}")
    return split_lines


def _matching_paren_index(text: str, open_index: int) -> int | None:
    depth = 0
    in_string = False
    index = open_index
    while index < len(text):
        char = text[index]
        if in_string:
            if char == "\\":
                index += 1
            elif char == '"':
                in_string = False
            index += 1
            continue
        if char == '"':
            in_string = True
        elif char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
        index += 1
    return None


def _split_top_level_commas(text: str) -> list[str]:
    parts: list[str] = []
    start = 0
    paren_depth = 0
    brace_depth = 0
    bracket_depth = 0
    in_string = False
    i = 0
    while i < len(text):
        char = text[i]
        if in_string:
            if char == "\\":
                i += 2
                continue
            if char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            in_string = True
        elif char == "(":
            paren_depth += 1
        elif char == ")":
            paren_depth -= 1
        elif char == "{":
            brace_depth += 1
        elif char == "}":
            brace_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif (
            char == ","
            and paren_depth == 0
            and brace_depth == 0
            and bracket_depth == 0
        ):
            parts.append(text[start:i])
            start = i + 1
        i += 1
    parts.append(text[start:])
    return parts


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BobLib Modelica formatter/linter")
    parser.add_argument(
        "command",
        choices=("check", "format", "diff"),
        help="Check formatting, rewrite files, or print a unified formatting diff.",
    )
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument(
        "--max-line-length",
        type=int,
        default=DEFAULT_MAX_LINE_LENGTH,
        help="Line length at which simple component modifier calls are wrapped.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    paths = tuple(args.paths)
    if args.command == "format":
        changed = format_paths(paths, max_line_length=args.max_line_length)
        for path in changed:
            print(f"formatted {path}")
        return 0

    if args.command == "diff":
        print(diff_paths(paths, max_line_length=args.max_line_length), end="")
        return 0

    diagnostics = check_paths(paths, max_line_length=args.max_line_length)
    for diagnostic in diagnostics:
        print(
            f"{diagnostic.path}:{diagnostic.line}: "
            f"{diagnostic.code} {diagnostic.message}"
        )
    return 1 if diagnostics else 0


if __name__ == "__main__":
    raise SystemExit(main())
