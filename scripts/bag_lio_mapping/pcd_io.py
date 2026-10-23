"""Minimal streaming PCD output used by bag mapping stages."""

from pathlib import Path
import shutil

import numpy as np


class BinaryPcdWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.temporary = path.with_suffix(path.suffix + ".points.tmp")
        self.handle = self.temporary.open("wb")
        self.count = 0

    def write(self, points: np.ndarray) -> None:
        xyz = np.ascontiguousarray(points, dtype=np.float32)
        self.handle.write(xyz.tobytes())
        self.count += len(xyz)

    def close(self) -> None:
        self.handle.close()
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n"
            "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {self.count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {self.count}\nDATA binary\n"
        )
        with self.path.open("wb") as destination, self.temporary.open("rb") as source:
            destination.write(header.encode("ascii"))
            shutil.copyfileobj(source, destination, length=4 * 1024 * 1024)
        self.temporary.unlink()
