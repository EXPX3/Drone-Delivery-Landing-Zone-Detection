#!/usr/bin/env python3
"""Build the static Plotly scene used by the synchronized bag replay server."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import open3d as o3d
import plotly.graph_objects as go
from PIL import Image


COLORS = {"safest": "#008000", "safe": "#FFA500", "risky": "#FF0000", "top30": "#90EE90"}


def grouped_zone_traces(csv_path: Path) -> list[go.Scatter3d]:
    groups: dict[str, list[dict[str, str]]] = {name: [] for name in COLORS}
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rank = int(row.get("rank_risk", row.get("rank", "0")))
            group = "top30" if rank <= 30 else row.get("risk_category", "safest")
            groups.setdefault(group, []).append(row)
    traces = []
    for category, rows in groups.items():
        if not rows:
            continue
        line_x: list[float | None] = []
        line_y: list[float | None] = []
        line_z: list[float | None] = []
        center_x, center_y, center_z, labels = [], [], [], []
        angles = np.linspace(0.0, 2.0 * np.pi, 36)
        for row in rows:
            x, y, z, radius = (float(row[key]) for key in ("x", "y", "z", "radius"))
            line_x.extend((x + radius * np.cos(angles)).tolist() + [None])
            line_y.extend((y + radius * np.sin(angles)).tolist() + [None])
            line_z.extend(np.full_like(angles, z + 0.15).tolist() + [None])
            center_x.append(x)
            center_y.append(y)
            center_z.append(z + 0.2)
            labels.append(f"LZ {row.get('rank_risk', row.get('rank', '?'))}")
        color = COLORS.get(category, "#ffffff")
        display_name = "Top 30" if category == "top30" else category.title()
        traces.append(go.Scatter3d(
            x=line_x, y=line_y, z=line_z, mode="lines", name=f"{display_name} footprints",
            line={"color": color, "width": 3}, hoverinfo="skip", visible=True,
        ))
        traces.append(go.Scatter3d(
            x=center_x, y=center_y, z=center_z, mode="markers", name=f"{display_name} centers",
            marker={"color": color, "size": 2, "opacity": 0.75}, text=labels, hovertemplate="%{text}<extra></extra>",
        ))
    return traces


def orthophoto_trace(path: Path, bounds: tuple[float, float, float, float], z: float, stride: int) -> go.Scatter3d:
    pixels = np.asarray(Image.open(path).convert("RGB"))[::stride, ::stride]
    rows, columns = pixels.shape[:2]
    min_x, min_y, max_x, max_y = bounds
    xx, yy = np.meshgrid(np.linspace(min_x, max_x, columns), np.linspace(max_y, min_y, rows))
    colors = [f"rgb({r},{g},{b})" for r, g, b in pixels.reshape(-1, 3)]
    return go.Scatter3d(
        x=xx.ravel(), y=yy.ravel(), z=np.full(xx.size, z), mode="markers", name="Orthophoto",
        marker={"size": 2, "color": colors, "opacity": 0.95}, hoverinfo="skip",
    )


def page_html(figure: go.Figure) -> str:
    plot = figure.to_html(full_html=False, include_plotlyjs=True, div_id="lz-scene",
                          config={"responsive": True, "displaylogo": False})
    return f"""<!doctype html>
<html><head><meta charset="utf-8"><title>LZ synchronized bag replay</title>
<style>
html,body{{height:100%;margin:0;background:#101418;color:#edf2f7;font:14px Arial,sans-serif;overflow:hidden}}
#app{{display:grid;grid-template-columns:minmax(0,1fr) 400px;height:100%}}
#scene{{min-width:0}} #lz-scene{{width:100%;height:100%}}
#side{{border-left:1px solid #39424c;background:#171d23;display:flex;flex-direction:column;min-width:0}}
#side h1{{font-size:17px;margin:16px 16px 10px;letter-spacing:0}}
#camera{{width:100%;aspect-ratio:16/9;object-fit:contain;background:#050607}}
#status{{padding:12px 16px;line-height:1.55;color:#cbd5e1;white-space:pre-line;overflow:auto}}
#controls{{display:flex;gap:8px;padding:0 16px 14px;align-items:center}}
button,select{{background:#27313b;color:#fff;border:1px solid #475569;padding:7px 10px;border-radius:4px}}
.legend{{padding:0 16px;color:#aab6c3;font-size:12px}}
.swatch{{display:inline-block;width:10px;height:10px;margin:0 5px 0 12px}}
@media(max-width:900px){{#app{{grid-template-columns:1fr;grid-template-rows:58% 42%}}#side{{border-left:0;border-top:1px solid #39424c}}}}
</style></head><body><div id="app"><main id="scene">{plot}</main><aside id="side">
<h1>Camera and bag replay</h1><img id="camera" alt="Waiting for camera frame">
<div id="controls"><button id="play">Pause</button><button id="restart">Restart</button>
<select id="rate"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select></div>
<div class="legend"><span class="swatch" style="background:#90EE90"></span>Top 30
<span class="swatch" style="background:#008000"></span>Safest
<span class="swatch" style="background:#FFA500"></span>Safe
<span class="swatch" style="background:#FF0000"></span>Risky</div><div id="status">Preparing bag...</div>
</aside></div><script>
const graph=document.getElementById('lz-scene'); let lastSeq=-1; let playing=true; let updating=false; let cameraUrl=null;
function trace(name){{return graph.data.findIndex(item=>item.name===name)}}
async function control(command,value=''){{await fetch(`/api/control?cmd=${{command}}&value=${{value}}`,{{method:'POST'}})}}
document.getElementById('play').onclick=async()=>{{playing=!playing;await control(playing?'play':'pause');document.getElementById('play').textContent=playing?'Pause':'Play'}};
document.getElementById('restart').onclick=()=>control('restart');
document.getElementById('rate').onchange=e=>control('rate',e.target.value);
async function update(){{if(updating)return;updating=true;try{{const s=await (await fetch('/api/state',{{cache:'no-store'}})).json();
const imageIds=(s.image_visible_zone_ranks||[]).join(', '),frustumIds=(s.frustum_visible_zone_ranks||[]).join(', ');
const match=s.projection_sets_match===true?'YES':(s.projection_sets_match===false?'NO':'waiting');
document.getElementById('status').textContent=`${{s.status}}\nBag time: ${{(s.bag_time_s||0).toFixed(3)}} s\nSynchronized seq: ${{s.seq||0}}\nROS image stamp: ${{s.image_timestamp_ns||'-'}} ns\nImage LZs: ${{(s.image_visible_zone_ranks||[]).length}}\nFrustum LZs: ${{(s.frustum_visible_zone_ranks||[]).length}}\nIndependent set match: ${{match}}\nChecked frames: ${{s.checked_frames||0}}\nMismatch frames: ${{s.mismatch_frames||0}}\nImage-only: ${{(s.image_only_zone_ranks||[]).join(', ')||'-'}}\nFrustum-only: ${{(s.frustum_only_zone_ranks||[]).join(', ')||'-'}}\nImage IDs: ${{imageIds}}\nFrustum IDs: ${{frustumIds}}\nReplay frame: ${{s.frame_number||0}}`;
if(s.seq!==lastSeq && s.camera_position){{const response=await fetch(`/api/frame.jpg?seq=${{s.seq}}`,{{cache:'no-store'}});if(!response.ok)throw new Error('Exact frame expired');const blob=await response.blob();
const trajectory=trace('UAV trajectory'),uav=trace('UAV position'),frustum=trace('Camera footprint'),rays=trace('Camera frustum'),visible=trace('Frustum-visible LZ highlight');
await Plotly.restyle(graph,{{
x:[s.trajectory.x,[s.uav_position[0]],s.footprint.x,s.frustum.x,s.visible_lzs.x],
y:[s.trajectory.y,[s.uav_position[1]],s.footprint.y,s.frustum.y,s.visible_lzs.y],
z:[s.trajectory.z,[s.uav_position[2]],s.footprint.z,s.frustum.z,s.visible_lzs.z]
}},[trajectory,uav,frustum,rays,visible]);
const nextUrl=URL.createObjectURL(blob);document.getElementById('camera').src=nextUrl;if(cameraUrl)URL.revokeObjectURL(cameraUrl);cameraUrl=nextUrl;lastSeq=s.seq;}}
}}catch(error){{document.getElementById('status').textContent=`Synchronization retry: ${{error.message}}`;}}finally{{updating=false;}}}}
setInterval(update,150);update();
</script></body></html>"""


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517.pcd")
    parser.add_argument("--zones", type=Path, default=root / "pcds/region_growing_landing_zones_max10000_radius2_5_to3_risk_ranked.csv")
    parser.add_argument("--orthophoto", type=Path, default=root / "pcds/696_5328_cropped_by_bayernatlas_20260812084517_satellite_overlay_dop20.png")
    parser.add_argument("--output", type=Path, default=root / "pcds/region_growing_landing_zones_live_bag_replay.html")
    parser.add_argument("--max-points", type=int, default=180000)
    parser.add_argument("--orthophoto-stride", type=int, default=5)
    args = parser.parse_args()
    cloud = o3d.io.read_point_cloud(str(args.pcd))
    points = np.asarray(cloud.points)
    if not len(points):
        raise RuntimeError(f"Empty point cloud: {args.pcd}")
    if len(points) > args.max_points:
        keep = np.random.default_rng(17).choice(len(points), args.max_points, replace=False)
        points = points[keep]
    full_points = np.asarray(cloud.points)
    mins, maxs = full_points.min(axis=0), full_points.max(axis=0)
    bounds = (float(mins[0] - 10), float(mins[1] - 10), float(maxs[0] + 10), float(maxs[1] + 10))
    figure = go.Figure()
    if args.orthophoto.exists():
        figure.add_trace(orthophoto_trace(args.orthophoto, bounds, float(mins[2] - 0.5), args.orthophoto_stride))
    figure.add_trace(go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers", name="Point cloud",
        marker={"size": 1.2, "color": points[:, 2], "colorscale": "Viridis", "opacity": 0.65}, hoverinfo="skip",
    ))
    for zone_trace in grouped_zone_traces(args.zones):
        figure.add_trace(zone_trace)
    figure.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="lines", name="UAV trajectory", line={"color": "#29b6f6", "width": 6}))
    figure.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="markers", name="UAV position", marker={"color": "#ffffff", "size": 6, "symbol": "diamond"}))
    figure.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="lines", name="Camera footprint", line={"color": "#00e5ff", "width": 7}))
    figure.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="lines", name="Camera frustum", line={"color": "#80deea", "width": 3}))
    figure.add_trace(go.Scatter3d(
        x=[], y=[], z=[], mode="lines", name="Frustum-visible LZ highlight",
        line={"color": "#FFB6C1", "width": 10}, hoverinfo="skip",
    ))
    figure.update_layout(
        scene={"xaxis_title": "UTM Easting", "yaxis_title": "UTM Northing", "zaxis_title": "Elevation", "aspectmode": "data"},
        margin={"l": 0, "r": 0, "t": 0, "b": 0}, paper_bgcolor="#101418", legend={"bgcolor": "rgba(20,25,30,.8)"},
    )
    args.output.write_text(page_html(figure), encoding="utf-8")
    print(f"Wrote replay viewer: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
