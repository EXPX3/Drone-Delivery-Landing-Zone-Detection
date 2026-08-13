#!/usr/bin/env python3
"""Crop a LAZ point cloud with a GPX polygon and save PCD + polygon mesh.

Expected input layout:
  Drone-Delivery-Landing-Zone-Detection/pcds/*.laz
  Drone-Delivery-Landing-Zone-Detection/pcds/*.gpx

Dependencies:
  pip install laspy[lazrs] numpy pyproj open3d

Example:
  python3 scripts/crop_laz_with_gpx.py
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import laspy
import numpy as np
from pyproj import CRS, Transformer


ROOT = Path(__file__).resolve().parents[1]
PCDS_DIR = ROOT / "pcds"


def single_file(directory: Path, suffix: str) -> Path:
    files = sorted(directory.glob(f"*{suffix}"))
    if not files:
        raise FileNotFoundError(f"No {suffix} file found in {directory}")
    if len(files) > 1:
        raise ValueError(
            f"Multiple {suffix} files found in {directory}. Pass --{suffix[1:]} explicitly."
        )
    return files[0]


def read_gpx_polygon(gpx_path: Path) -> np.ndarray:
    root = ET.parse(gpx_path).getroot()
    points: list[tuple[float, float]] = []
    for element in root.iter():
        if element.tag.endswith("trkpt") or element.tag.endswith("wpt") or element.tag.endswith("rtept"):
            lat = element.attrib.get("lat")
            lon = element.attrib.get("lon")
            if lat is not None and lon is not None:
                points.append((float(lon), float(lat)))

    if len(points) < 3:
        raise ValueError(f"{gpx_path} does not contain at least three GPX points")

    polygon = np.asarray(points, dtype=np.float64)
    if not np.allclose(polygon[0], polygon[-1]):
        polygon = np.vstack([polygon, polygon[0]])
    return polygon


def infer_las_crs(las: laspy.LasData, epsg: int | None) -> CRS:
    if epsg is not None:
        return CRS.from_epsg(epsg)

    crs = las.header.parse_crs()
    if crs is None:
        raise ValueError(
            "Could not infer CRS from LAZ header. Re-run with --epsg, for example --epsg 25832."
        )
    return CRS.from_user_input(crs)


def points_in_polygon(x: np.ndarray, y: np.ndarray, polygon_xy: np.ndarray) -> np.ndarray:
    """Vectorized ray-casting point-in-polygon test."""
    inside = np.zeros(x.shape, dtype=bool)
    px = polygon_xy[:, 0]
    py = polygon_xy[:, 1]

    for i in range(len(polygon_xy) - 1):
        x1, y1 = px[i], py[i]
        x2, y2 = px[i + 1], py[i + 1]
        crosses = (y1 > y) != (y2 > y)
        x_intersect = (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-30) + x1
        inside ^= crosses & (x < x_intersect)
    return inside


ASPRS_CLASSES = {
    0: "created, never classified",
    1: "unclassified",
    2: "ground",
    3: "low vegetation",
    4: "medium vegetation",
    5: "high vegetation",
    6: "building",
    7: "low point/noise",
    9: "water",
    17: "bridge deck",
    18: "high noise",
    20: "ignored ground",
    22: "temporal exclusion",
}


def write_ascii_pcd(path: Path, xyz: np.ndarray, classification: np.ndarray | None = None) -> None:
    fields = "x y z"
    size = "4 4 4"
    types = "F F F"
    count = "1 1 1"
    data = xyz
    fmt = "%.6f %.6f %.6f"
    if classification is not None:
        fields += " classification"
        size += " 4"
        types += " F"
        count += " 1"
        data = np.column_stack([xyz, classification.astype(np.float32)])
        fmt += " %.0f"

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            f"FIELDS {fields}",
            f"SIZE {size}",
            f"TYPE {types}",
            f"COUNT {count}",
            f"WIDTH {len(xyz)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(xyz)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as handle:
        handle.write(header + "\n")
        np.savetxt(handle, data, fmt=fmt)


def write_obj_mesh(path: Path, polygon_xy: np.ndarray, z_min: float, z_max: float) -> None:
    vertices_2d = polygon_xy[:-1] if np.allclose(polygon_xy[0], polygon_xy[-1]) else polygon_xy
    z_span = max(z_max - z_min, 1.0)
    bottom_z = z_min - 0.02 * z_span
    top_z = z_max + 0.08 * z_span
    with path.open("w", encoding="ascii") as handle:
        handle.write("# Extruded GPX crop boundary mesh\n")
        for z in (bottom_z, top_z):
            for vertex in vertices_2d:
                handle.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {z:.6f}\n")
        n = len(vertices_2d)
        for idx in range(n):
            nxt = (idx + 1) % n
            handle.write(f"f {idx + 1} {nxt + 1} {n + nxt + 1}\n")
            handle.write(f"f {idx + 1} {n + nxt + 1} {n + idx + 1}\n")


def bounds_text(name: str, xy: np.ndarray) -> str:
    return (
        f"{name}: "
        f"x=[{float(np.min(xy[:, 0])):.3f}, {float(np.max(xy[:, 0])):.3f}], "
        f"y=[{float(np.min(xy[:, 1])):.3f}, {float(np.max(xy[:, 1])):.3f}]"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laz", type=Path, default=None, help="Input .laz file")
    parser.add_argument("--gpx", type=Path, default=None, help="Input .gpx polygon/track file")
    parser.add_argument("--epsg", type=int, default=None, help="EPSG code for the LAZ CRS if missing")
    parser.add_argument("--out-prefix", default=None, help="Output filename prefix")
    parser.add_argument(
        "--include-classification",
        action="store_true",
        help="Write a PCD with x y z classification fields instead of x y z only",
    )
    parser.add_argument(
        "--class-filter",
        type=int,
        nargs="+",
        default=None,
        help="Keep only these LAS classification codes, e.g. --class-filter 2 6",
    )
    args = parser.parse_args()

    laz_path = args.laz or single_file(PCDS_DIR, ".laz")
    gpx_path = args.gpx or single_file(PCDS_DIR, ".gpx")
    prefix = args.out_prefix or f"{laz_path.stem}_cropped_by_{gpx_path.stem}"

    print(f"Reading LAZ: {laz_path}")
    las = laspy.read(laz_path)
    source_crs = infer_las_crs(las, args.epsg)
    transformer = Transformer.from_crs("EPSG:4326", source_crs, always_xy=True)

    lon_lat = read_gpx_polygon(gpx_path)
    poly_x, poly_y = transformer.transform(lon_lat[:, 0], lon_lat[:, 1])
    polygon_xy = np.column_stack([poly_x, poly_y])

    xyz = np.column_stack([las.x, las.y, las.z]).astype(np.float64, copy=False)
    mask = points_in_polygon(xyz[:, 0], xyz[:, 1], polygon_xy)
    classifications = None
    if "classification" in las.point_format.dimension_names:
        classifications = np.asarray(las.classification)
        if args.class_filter is not None:
            mask &= np.isin(classifications, args.class_filter)
    cropped = xyz[mask]
    mesh_path = PCDS_DIR / f"{prefix}_gpx_mesh.obj"
    z_source = cropped if len(cropped) else xyz
    write_obj_mesh(mesh_path, polygon_xy, float(np.min(z_source[:, 2])), float(np.max(z_source[:, 2])))
    if len(cropped) == 0:
        las_xy = np.array(
            [
                [float(np.min(xyz[:, 0])), float(np.min(xyz[:, 1]))],
                [float(np.max(xyz[:, 0])), float(np.max(xyz[:, 1]))],
            ]
        )
        raise RuntimeError(
            "Crop returned 0 points.\n"
            f"{bounds_text('LAZ bounds', las_xy)}\n"
            f"{bounds_text('GPX bounds', polygon_xy)}\n"
            f"Wrote mesh for inspection: {mesh_path}\n"
            "Use a LAZ tile that overlaps the GPX polygon, or pass the correct --epsg if this CRS is wrong."
        )

    pcd_path = PCDS_DIR / f"{prefix}.pcd"
    cropped_classes = classifications[mask] if args.include_classification and classifications is not None else None
    write_ascii_pcd(pcd_path, cropped, cropped_classes)

    print(f"Source CRS: {source_crs.to_string()}")
    print(f"Cropped {len(cropped)} / {len(xyz)} points")
    if classifications is not None:
        values, counts = np.unique(classifications[mask], return_counts=True)
        print("Cropped classification counts:")
        for value, count in zip(values, counts):
            label = ASPRS_CLASSES.get(int(value), "unknown/reserved")
            print(f"  {int(value):2d} {label}: {int(count)}")
    print(f"Wrote PCD:  {pcd_path}")
    print(f"Wrote mesh: {mesh_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
