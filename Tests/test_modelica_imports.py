"""Keep library imports direct and reserve aliases for reviewed exceptions."""
import re
from pathlib import Path

from modelica_parameters import library_files, masked

ROOT = Path(__file__).resolve().parents[1]


def test_imports_do_not_rename_packages_or_types():
    # Add a documented exception here only when a direct import is ambiguous.
    exceptions = set()
    aliases = set()
    for path in library_files(ROOT):
        for match in re.finditer(r"\bimport\s+(\w+)\s*=\s*([\w.]+)", masked(path.read_text())):
            aliases.add((str(path.relative_to(ROOT)), match[1], match[2]))
    assert aliases == exceptions, sorted(aliases - exceptions)
