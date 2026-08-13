#!/usr/bin/env python3
"""Visualize a cropped PCD point cloud with optional GPX and orthophoto layers.

Dependencies:
  pip install open3d numpy plotly pillow requests

Example:
  python3 scripts/visualize_pcd_gpx_mesh.py
  python3 scripts/visualize_pcd_gpx_mesh.py --pcd pcds/result.pcd --mesh pcds/result_gpx_mesh.obj
  python3 scripts/visualize_pcd_gpx_mesh.py --pcd pcds/result.pcd --html pcds/result.html --orthophoto image.png
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import requests
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
PCDS_DIR = ROOT / "pcds"


def newest_file(directory: Path, pattern: str) -> Path:
    files = sorted(directory.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
    if not files:
        raise FileNotFoundError(f"No file matching {pattern} found in {directory}")
    return files[0]


def optional_newest_file(directory: Path, pattern: str) -> Path | None:
    try:
        return newest_file(directory, pattern)
    except FileNotFoundError:
        return None


def mesh_edges(mesh: o3d.geometry.TriangleMesh) -> tuple[list[float | None], list[float | None], list[float | None]]:
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    edges = set()
    for tri in triangles:
        for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            edges.add(tuple(sorted((int(a), int(b)))))

    xs: list[float | None] = []
    ys: list[float | None] = []
    zs: list[float | None] = []
    for a, b in sorted(edges):
        xs.extend([float(vertices[a, 0]), float(vertices[b, 0]), None])
        ys.extend([float(vertices[a, 1]), float(vertices[b, 1]), None])
        zs.extend([float(vertices[a, 2]), float(vertices[b, 2]), None])
    return xs, ys, zs


def pcd_bounds(points: np.ndarray) -> tuple[float, float, float, float, float, float]:
    mins = np.min(points, axis=0)
    maxs = np.max(points, axis=0)
    return float(mins[0]), float(mins[1]), float(mins[2]), float(maxs[0]), float(maxs[1]), float(maxs[2])


def fetch_bayern_dop20(
    image_path: Path,
    bounds: tuple[float, float, float, float],
    width: int,
    height: int,
) -> None:
    minx, miny, maxx, maxy = bounds
    params = {
        "SERVICE": "WMS",
        "VERSION": "1.3.0",
        "REQUEST": "GetMap",
        "LAYERS": "by_dop20c",
        "STYLES": "",
        "CRS": "EPSG:25832",
        "BBOX": f"{minx},{miny},{maxx},{maxy}",
        "WIDTH": str(width),
        "HEIGHT": str(height),
        "FORMAT": "image/png",
        "TRANSPARENT": "FALSE",
    }
    response = requests.get(
        "https://geoservices.bayern.de/od/wms/dop/v1/dop20",
        params=params,
        timeout=60,
    )
    response.raise_for_status()
    content_type = response.headers.get("content-type", "")
    if "image" not in content_type.lower():
        raise RuntimeError(f"WMS did not return an image. Content-Type: {content_type}")
    image_path.write_bytes(response.content)


def orthophoto_trace(
    image_path: Path,
    bounds: tuple[float, float, float, float],
    z: float,
    stride: int,
):
    import plotly.graph_objects as go

    image = Image.open(image_path).convert("RGB")
    arr = np.asarray(image)
    arr = arr[::stride, ::stride]
    rows, cols, _ = arr.shape
    minx, miny, maxx, maxy = bounds
    xs = np.linspace(minx, maxx, cols)
    ys = np.linspace(maxy, miny, rows)
    xx, yy = np.meshgrid(xs, ys)
    colors = [f"rgb({r},{g},{b})" for r, g, b in arr.reshape(-1, 3)]
    return go.Scatter3d(
        x=xx.ravel(),
        y=yy.ravel(),
        z=np.full(xx.size, z),
        mode="markers",
        name="Bayern DOP20 orthophoto",
        marker={"size": 2.0, "color": colors, "opacity": 1.0},
        hoverinfo="skip",
    )


def landing_zone_traces(csv_path: Path):
    import plotly.graph_objects as go

    traces = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rank = int(row["rank"])
            cx = float(row["x"])
            cy = float(row["y"])
            cz = float(row["z"])
            radius = float(row["radius"])
            theta = np.linspace(0.0, 2.0 * np.pi, 96)
            xs = cx + radius * np.cos(theta)
            ys = cy + radius * np.sin(theta)
            zs = np.full_like(xs, cz + 0.35)
            traces.append(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines",
                    name=f"LZ {rank}: r={radius:.2f} m",
                    line={"color": "lime", "width": 8},
                )
            )
            traces.append(
                go.Scatter3d(
                    x=[cx],
                    y=[cy],
                    z=[cz + 0.5],
                    mode="markers+text",
                    name=f"LZ {rank} center",
                    marker={"size": 5, "color": "black"},
                    text=[str(rank)],
                    textposition="top center",
                    showlegend=False,
                )
            )
    return traces


def write_html_viewer(
    output_path: Path,
    point_cloud: o3d.geometry.PointCloud,
    mesh: o3d.geometry.TriangleMesh | None,
    max_points: int,
    orthophoto_path: Path | None,
    orthophoto_stride: int,
    show_gpx: bool,
    landing_zones_csv: Path | None,
) -> None:
    import plotly.graph_objects as go

    points = np.asarray(point_cloud.points)
    minx, miny, minz, maxx, maxy, _maxz = pcd_bounds(points)
    margin = 10.0
    image_bounds = (minx - margin, miny - margin, maxx + margin, maxy + margin)
    if len(points) > max_points:
        rng = np.random.default_rng(7)
        keep = np.sort(rng.choice(len(points), size=max_points, replace=False))
        points = points[keep]

    fig = go.Figure()
    if orthophoto_path is not None:
        fig.add_trace(orthophoto_trace(orthophoto_path, image_bounds, minz - 0.5, orthophoto_stride))
    fig.add_trace(
        go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            mode="markers",
            name="Point cloud",
            marker={"size": 1.4, "color": points[:, 2], "colorscale": "Viridis", "opacity": 0.75},
        )
    )
    if mesh is not None:
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)
        edge_x, edge_y, edge_z = mesh_edges(mesh)
        gpx_visibility: bool | str = True if show_gpx else "legendonly"
        fig.add_trace(
            go.Mesh3d(
                x=vertices[:, 0],
                y=vertices[:, 1],
                z=vertices[:, 2],
                i=triangles[:, 0],
                j=triangles[:, 1],
                k=triangles[:, 2],
                name="GPX area",
                color="red",
                opacity=0.12,
                visible=gpx_visibility,
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=edge_x,
                y=edge_y,
                z=edge_z,
                mode="lines",
                name="GPX boundary",
                line={"color": "yellow", "width": 8},
                visible=gpx_visibility,
            )
        )
    if landing_zones_csv is not None:
        for trace in landing_zone_traces(landing_zones_csv):
            fig.add_trace(trace)
    fig.update_layout(
        scene={
            "xaxis_title": "UTM x",
            "yaxis_title": "UTM y",
            "zaxis_title": "z",
            "aspectmode": "data",
        },
        margin={"l": 0, "r": 0, "t": 30, "b": 0},
        title="Landing Zones + Point Cloud + Orthophoto",
        legend={
            "title": {"text": "Layers"},
            "itemsizing": "constant",
            "x": 0.01,
            "y": 0.99,
            "bgcolor": "rgba(255,255,255,0.8)",
        },
    )
    fig.write_html(output_path, include_plotlyjs=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, default=None, help="PCD file to visualize")
    parser.add_argument("--mesh", type=Path, default=None, help="Optional OBJ GPX mesh file to overlay")
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--check-only", action="store_true", help="Load files and print stats without opening a GUI")
    parser.add_argument("--html", type=Path, default=None, help="Write a browser-based HTML viewer instead of opening Open3D")
    parser.add_argument("--html-max-points", type=int, default=200000, help="Maximum PCD points to include in HTML")
    parser.add_argument("--orthophoto", type=Path, default=None, help="Orthophoto image to use as an HTML base layer")
    parser.add_argument("--fetch-bayern-dop20", action="store_true", help="Fetch a Bayern DOP20 orthophoto base layer")
    parser.add_argument("--orthophoto-size", type=int, default=1200, help="WMS image width/height in pixels")
    parser.add_argument("--orthophoto-stride", type=int, default=4, help="Use every Nth orthophoto pixel in HTML")
    parser.add_argument("--show-gpx", action="store_true", help="Show GPX area by default in the HTML viewer")
    parser.add_argument("--no-gpx", action="store_true", help="Do not include a GPX mesh layer in the HTML viewer")
    parser.add_argument("--landing-zones-csv", type=Path, default=None, help="CSV exported by the C++ landing-zone detector")
    args = parser.parse_args()

    pcd_path = args.pcd or newest_file(PCDS_DIR, "*.pcd")
    mesh_path = None if args.no_gpx else args.mesh or optional_newest_file(PCDS_DIR, "*_gpx_mesh.obj")

    point_cloud = o3d.io.read_point_cloud(str(pcd_path))
    if point_cloud.is_empty():
        raise RuntimeError(f"Point cloud is empty or unreadable: {pcd_path}")
    point_cloud.paint_uniform_color([0.15, 0.55, 1.0])

    mesh = None
    wireframe = None
    if mesh_path is not None:
        mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        if mesh.is_empty():
            raise RuntimeError(f"Mesh is empty or unreadable: {mesh_path}")
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([1.0, 0.25, 0.1])
        wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
        wireframe.paint_uniform_color([1.0, 0.95, 0.1])

    print(f"Visualizing PCD:  {pcd_path}")
    print(f"Visualizing mesh: {mesh_path or 'disabled'}")
    print(f"PCD points:       {len(point_cloud.points)}")
    if mesh is not None:
        print(f"Mesh vertices:    {len(mesh.vertices)}")
        print(f"Mesh triangles:   {len(mesh.triangles)}")
    if args.html is not None:
        orthophoto_path = args.orthophoto
        if args.fetch_bayern_dop20:
            points = np.asarray(point_cloud.points)
            minx, miny, _minz, maxx, maxy, _maxz = pcd_bounds(points)
            margin = 10.0
            orthophoto_path = args.html.with_name(args.html.stem + "_dop20.png")
            fetch_bayern_dop20(
                orthophoto_path,
                (minx - margin, miny - margin, maxx + margin, maxy + margin),
                args.orthophoto_size,
                args.orthophoto_size,
            )
            print(f"Wrote orthophoto:  {orthophoto_path}")
        write_html_viewer(
            args.html,
            point_cloud,
            mesh,
            args.html_max_points,
            orthophoto_path,
            args.orthophoto_stride,
            args.show_gpx,
            args.landing_zones_csv,
        )
        print(f"Wrote HTML viewer: {args.html}")
        return 0
    if args.check_only:
        return 0
    print("Close the Open3D window to exit.")

    visualizer = o3d.visualization.Visualizer()
    if not visualizer.create_window(window_name="GPX Mesh + Cropped PCD", width=1280, height=800):
        raise RuntimeError(
            "Open3D could not create an OpenGL window. On Wayland, try running with "
            "XDG_SESSION_TYPE=x11 or QT_QPA_PLATFORM=xcb, or run from an X11 session."
        )
    visualizer.add_geometry(point_cloud)
    if mesh is not None and wireframe is not None:
        visualizer.add_geometry(mesh)
        visualizer.add_geometry(wireframe)
    render = visualizer.get_render_option()
    if render is not None:
        render.point_size = args.point_size
        render.mesh_show_back_face = True
    visualizer.run()
    visualizer.destroy_window()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
