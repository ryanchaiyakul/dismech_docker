import os
import pydoc
import subprocess
import sys
import tempfile
from pathlib import Path

import py_dismech
from fastmcp import FastMCP

WORKDIR = os.environ.get("DISMECH_WORKDIR", "/app/output")
MAX_CHARS = 50_000

mcp = FastMCP(
    "dismech",
    instructions="Read api_reference, then pass a complete py_dismech script to run_script. "
    "Relative paths write to the output volume.",
)


def _tail(out: str | bytes | None) -> str:
    if isinstance(out, bytes):
        out = out.decode(errors="replace")
    return (out or "")[-MAX_CHARS:]


@mcp.tool()
def run_script(code: str, timeout_s: int = 600) -> dict:
    """Run a complete Python script. Returns exit_code, stdout, stderr (last 50k chars each) and timed_out."""
    with tempfile.TemporaryDirectory() as tmp:
        script = Path(tmp, "script.py")
        script.write_text(code)
        try:
            p = subprocess.run(
                [sys.executable, script], cwd=WORKDIR, capture_output=True, text=True, timeout=timeout_s, check=False
            )
            return {"exit_code": p.returncode, "stdout": _tail(p.stdout), "stderr": _tail(p.stderr), "timed_out": False}
        except subprocess.TimeoutExpired as e:
            return {"exit_code": None, "stdout": _tail(e.stdout), "stderr": _tail(e.stderr), "timed_out": True}


@mcp.tool()
def api_reference() -> str:
    """Return the py_dismech API (classes, methods, signatures)."""
    return pydoc.render_doc(py_dismech.py_dismech, renderer=pydoc.plaintext)  # type: ignore


def main() -> None:
    mcp.run(show_banner=False)
