from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

import modelica_fmu_export as fmu_export


def _is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A killed process stays a zombie until its new parent reaps it.
    state = subprocess.run(
        ["ps", "-o", "stat=", "-p", str(pid)],
        check=False,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    return bool(state) and not state.startswith("Z")


@pytest.mark.skipif(os.name == "nt", reason="the FMU export does not support Windows")
def test_timeout_stops_the_jobs_a_command_started(tmp_path: Path) -> None:
    # Like make starting compilers: the command starts a child, then waits.
    pid_file = tmp_path / "child.pid"
    script = (
        "import subprocess, sys, time\n"
        "child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])\n"
        f"open({str(pid_file)!r}, 'w').write(str(child.pid))\n"
        "time.sleep(60)\n"
    )

    with pytest.raises(RuntimeError, match="timed out after 2 seconds"):
        fmu_export.run_command([sys.executable, "-c", script], cwd=tmp_path, timeout_s=2)

    child = int(pid_file.read_text())
    deadline = time.monotonic() + 5.0
    while _is_running(child) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert not _is_running(child), f"child process {child} survived the timeout"
