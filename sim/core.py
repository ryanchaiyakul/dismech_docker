from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re


OUTPUT_DIR = Path("/app/output")
NAME_PATTERN = re.compile(r"[A-Za-z0-9._-]+")


@dataclass(frozen=True)
class SimulationResult:
    output_path: Path
    stdout: str = ""


def validate_output_name(name: str) -> str:
    if not NAME_PATTERN.fullmatch(name):
        raise ValueError("name must contain only letters, numbers, dot, underscore, or hyphen")
    return name


def prepare_output_dir(output_dir: str | Path = OUTPUT_DIR) -> Path:
    resolved = Path(output_dir)
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved
