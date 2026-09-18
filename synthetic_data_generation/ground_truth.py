"""Strict JSONL annotations for generated LiDAR frames."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, TextIO


def annotation_path_for(output_bag: Path | str) -> Path:
    return Path(f"{Path(output_bag)}.annotations.jsonl")


class AnnotationWriter:
    """Exclusive, line-buffered JSONL writer that never overwrites a file."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        if self.path.exists():
            raise FileExistsError(f"Annotation path already exists: {self.path}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream: TextIO | None = None

    def open(self) -> None:
        self._stream = self.path.open("x", encoding="utf-8", newline="\n", buffering=1)

    def write(self, record: Mapping[str, Any]) -> None:
        if self._stream is None:
            raise RuntimeError("Annotation writer is not open")
        line = json.dumps(record, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
        self._stream.write(line + "\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
