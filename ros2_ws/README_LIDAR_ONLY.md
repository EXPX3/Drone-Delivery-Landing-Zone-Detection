# Live LiDAR landing-zone selection

This implementation is on branch `live_demo_cp_lidar_only`. It uses a fixed,
downward-facing LiDAR and has no camera, Gremsy, descent, or flight-control
command path. Its only navigation output is the selected horizontal location as
local ENU offset and WGS84 latitude/longitude.

## Modes

- `ranking` / `1`: continually show every candidate in final risk order.
- `sticky` / `2`: select the lowest-risk confirmed `SAFEST` or `SAFE` candidate,
  keep it bright while other candidates are dim, tolerate short detection
  dropouts, and replace it after a hard rejection or sustained risk degradation.

Candidates are `SAFEST`, `SAFE`, `RISKY`, or `UNKNOWN`. `UNKNOWN` means pending
temporal confirmation or insufficient clearance evidence; it is visible for
diagnosis but is never selectable. Each result also carries the explicit
evidence state `PENDING`, `CONFIRMED`, `INSUFFICIENT`, or `INVALID`.

The published `rank` is computed after the complete geometry risk score. Lower
risk is better; unscored candidates are last. The score combines obstacles,
obstacle clearance, relief, roughness, slope, and observed coverage.

## Supported sensor profiles

| Selection | Live cloud | Point time | Driver |
|---|---|---|---|
| `vlp16` | `/velodyne_points` | `time`, float32 seconds | ROS Velodyne |
| `os1_rev7` | `/ouster/points` | `t`, uint32 nanoseconds | pinned Ouster ROS; `LEGACY` UDP profile for FW 3.1 |
| `os1_rev8` | `/ouster/points` | `t`, uint32 nanoseconds | pinned Ouster ROS 0.15.1 |

The profile configures both the upstream live driver and downstream cloud
schema. Live use requires an explicit sensor. Bag use defaults to `auto`: VLP16
can be proven from its fields, while Ouster revision requires recorded metadata
with a recognizable hardware part number. If it cannot be proven, launch stops
and requires `--sensor os1_rev7` or `--sensor os1_rev8`; it never guesses from a
topic name or firmware version.

## Frame and telemetry contract

- `target_frame` must be a gravity-aligned local ENU frame: X east, Y north, Z up.
- A timestamped TF from each LiDAR cloud frame into `target_frame` is mandatory.
- The fixed LiDAR extrinsic must be supplied by the vehicle TF tree, or explicitly
  with `--static-lidar-transform`; no downward mounting angle is assumed.
- The pose and GNSS topics must describe the same vehicle reference at comparable
  timestamps. Defaults are `/mavros/local_position/pose` and
  `/mavros/global_position/global`.

For a valid sticky selection, the node subtracts the synchronized vehicle ENU
position from the candidate ENU position and applies that east/north displacement
to the synchronized GNSS fix using the WGS84 geodesic. GNSS status, timing, and
configured covariance limits are checked before publication.

## Build

From the repository root:

```bash
docker build -f docker/Dockerfile.live_demo_cargopack -t ddlzd-lidar-only:humble .
```

The image builds the pinned Rev8-capable Ouster driver, the Velodyne driver, and
the detector workspace.

## Run live

For the current Ethernet-connected Ouster rig, `eno1` must retain the host
address `169.254.50.1/16`. Before a full-rate run, apply the host UDP buffer
tuning once after boot:

```bash
sudo ./docker/configure_ouster_network.sh
```

Mode 1 with a VLP16:

```bash
./docker/run_landing_zones.sh live \
  --sensor vlp16 --velodyne-ip 192.168.1.201 \
  --mode 1 --target-frame map
```

Mode 2 with the current OS1-128 Rev7:

```bash
./docker/run_landing_zones.sh live \
  --sensor os1_rev7 --sensor-hostname 169.254.210.24 \
  --mode 2 --target-frame map
```

For the new hardware, change only `--sensor os1_rev8` and its hostname. The
default Ouster clock setting is `TIME_FROM_ROS_TIME`; use another setting only
when sensor, TF, pose, and GNSS timestamps share that clock.

If the calibrated fixed transform is not already published, add six values in
metres/radians:

```bash
--base-frame base_link --static-lidar-transform X Y Z ROLL PITCH YAW
```

## Run a bag

Automatic sensor resolution:

```bash
./docker/run_landing_zones.sh bag \
  --bag /absolute/path/to/bag --sensor auto \
  --mode 2 --target-frame map
```

Explicit resolution when the bag lacks Ouster hardware metadata:

```bash
./docker/run_landing_zones.sh bag \
  --bag /absolute/path/to/bag --sensor os1_rev7 \
  --point-cloud-topic /ouster/points \
  --mode 2 --target-frame map
```

Bag playback uses `/clock`. A cloud sequence is processed only once, so pausing
the bag cannot falsely increase temporal stability.

Press `Ctrl-C` to stop either workflow; the foreground launch and container are
stopped gracefully.

## Outputs

- `/landing_zone/live_landing_zone/zones`: fully ranked candidates and evidence.
- `/landing_zone/live_landing_zone/markers`: ranked or sticky visualization.
- `/landing_zone/live_landing_zone/selected`: selection state, local pose,
  vehicle-relative ENU, GNSS validity, latitude, longitude, and GNSS uncertainty.
- `/landing_zone/live_landing_zone/selected_gnss`: standard
  `geographic_msgs/GeoPointStamped`, published only for a valid locked selection.
- `/diagnostics`: fail-closed input, TF, and processing status.

No selected GNSS target is published in ranking mode, during a dropout/coasting
state, or when synchronized telemetry and GNSS validity checks fail.

## Native ROS launch entry points

The wrapper calls these launches and exposes the same values as ROS arguments:

```bash
ros2 launch ddlzd_ros live_lidar.launch.py sensor_model:=vlp16 ...
ros2 launch ddlzd_ros bag_landing_zones.launch.py bag_path:=/bag ...
ros2 launch ddlzd_ros live_fusion_only.launch.py sensor_model:=os1_rev7 ...
```

`live_ouster_gremsy.launch.py` remains only as an Ouster compatibility wrapper;
despite its historical filename, it starts no Gremsy or camera component.

## Build and test without Docker

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
