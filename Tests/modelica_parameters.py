"""Read parameter declarations without confusing strings or table rows with syntax."""
import re
from pathlib import Path

TRIVIA = re.compile(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*[\s\S]*?\*/')
PARAMETER = re.compile(
    r'(?m)^[ \t]*(?:(?:final|inner|outer|replaceable)\s+)*'
    r'parameter\s+([\w.]+)(?:\s*\[[^\]]*\])?\s+(\w+)'
)


def masked(source):
    chars = list(source)
    for match in TRIVIA.finditer(source):
        chars[match.start():match.end()] = [
            '\n' if char == '\n' else ' ' for char in match.group()
        ]
    return ''.join(chars)


def parameters(source):
    syntax = masked(source)
    strings = {m.start(): m.group() for m in TRIVIA.finditer(source)
               if m.group().startswith('"')}
    for match in PARAMETER.finditer(syntax):
        depth = 0
        annotation = None
        description = None
        for pos in range(match.end(), len(source)):
            char = syntax[pos]
            if depth == 0:
                if char == ';':
                    break
                if syntax.startswith('annotation', pos):
                    annotation = pos
                if pos in strings and annotation is None:
                    # A quoted binding or function argument is not a declaration comment.
                    before = source[match.end():pos].rstrip()
                    if not before.endswith(('=', '+', ',', '(', 'then', 'else')):
                        description = strings[pos]
            if char in '([{':
                depth += 1
            elif char in ')]}':
                depth -= 1
        else:
            raise ValueError(f'Unterminated parameter {match[2]}')
        prefix = syntax[match.start():match.end()]
        yield dict(name=match[2], type=match[1], start=match.start(), end=pos+1,
                   annotation=annotation, description=description,
                   final=bool(re.search(r'\bfinal\b', prefix)),
                   outer=bool(re.search(r'\bouter\b', prefix)))


def library_files(root):
    return sorted(p for folder in ('BobLib', 'Tests/BobLibTest')
                  for p in (Path(root)/folder).rglob('*.mo'))
