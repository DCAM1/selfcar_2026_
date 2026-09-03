# Reproducible VTD build and run

This repository's VTD path uses a digest-pinned Autoware CUDA/Jazzy base image
and builds the tracked source overlay into `/opt/selfcar_overlay`. It does not
mount source files or `.so` files from a developer home directory.

## New machine bootstrap

The host needs Ubuntu 24.04, Docker with BuildKit/Buildx, NVIDIA Container
Toolkit, and an X11 display. Obtain the external Autoware maps, ML models and
VTD installation separately; their expected locations are recorded in
`.env.example`.

The VTD bridge build reads only
`Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h` and
`Develop/Communication/Common/viRDBTypes.h` from `VTD_INSTALL_DIR`. The runtime
bridge additionally mounts the VTD installation for its simulator connection.

```bash
git clone https://github.com/DCAM1/selfcar_2026_.git
cd selfcar_2026_
git checkout chore/reproducible-vtd-source
cp .env.example .env
# Edit .env: set AUTOWARE_MAP_DIR, AUTOWARE_ML_MODELS_DIR,
# VTD_MAP_DIR, VTD_MAP_RELATIVE_PATH, VTD_INSTALL_DIR and DISPLAY.
./scripts/verify-reproducibility.sh
./scripts/build-vtd-image.sh
./vtd_bridge
./autoware routes/route_example.csv
```

`./autoware` checks that the external map/model directories and the selected
VTD map exist, enables temporary X11 access, runs the planning simulator with
the existing launch arguments, and removes the X11 access rule on exit. When a
CSV path is supplied, it loads that route as soon as the map and VTD
localization are ready.

## CSV routes

Route files require `seq,x,y` columns. Optional `z`, `yaw`, and `lanelet_id`
columns are accepted; `yaw` is a map-matching hint in radians and
`lanelet_id` is a persistent manual override. Rows are sorted by `seq`, which
must be unique and consecutive from 1. Sequence 1 is the expected vehicle
start, the rows in between are checkpoints, and the last sequence is the goal.

The setter never sends the raw CSV coordinates directly. For every row it
finds nearby road lanelets, compares their directed centerlines with the CSV
heading, and uses the routing graph to select one forward-connected sequence
across all rows. Each point is projected onto the selected centerline and its
yaw is replaced with the lanelet direction. Goal validation also checks that
the centered vehicle has lateral clearance inside the selected goal lanelet.

Start Autoware and load the example route automatically:

```bash
./autoware routes/route_example.csv
```

The VTD ego pose supplies Autoware's actual route start. To prevent silently
planning from the wrong location, the setter waits for localization and checks
that the vehicle is within 5 m of corrected sequence 1. It sends corrected
sequences 2 through N-1 as waypoints and corrected sequence N as the goal. An
`UNSET` route uses `/api/routing/set_route_points`; a `SET` route is replaced
without clearing it through `/api/routing/change_route_points`.

To replace a route on an Autoware instance that is already running, use a
second terminal:

```bash
./set_route /home/a/route_example.csv
```

The default RViz configuration displays four transient-local previews:

- `/debug/csv/raw_checkpoints`: red X markers for official CSV coordinates.
- `/debug/csv/candidate_lanelets`: yellow lanelet outlines.
- `/debug/csv/selected_lanelets`: the selected connected route in green.
- `/debug/csv/corrected_checkpoints`: blue waypoint dots/arrows and a blue
  star for the final goal.

Preview without changing Autoware:

```bash
./set_route official.csv --preview-only
```

To override an ambiguous row, request one or more sequence numbers and click
the desired road with RViz's **Publish Point** tool. The result is saved with
corrected coordinates, lanelet yaw, and fixed `lanelet_id` values:

```bash
./set_route official.csv \
  --override-seq 5 \
  --output-csv corrected_route.csv
```

If `--output-csv` is omitted during an override, the wrapper writes
`corrected_route.csv` in the current directory. Existing output files are
protected unless `--force-output` is supplied. Run `./set_route --help` for
search radius, direction threshold, preview, start-tolerance, and validation
options. CSV files are runtime inputs and do not require an image rebuild;
rebuild the image to update the copy used by `./autoware ROUTE.csv`.

## Overlay build

`scripts/build-vtd-image.sh` validates the immutable base-image digest and the
VTD RDB header, then invokes Buildx with the VTD installation as a read-only
named build context. The Dockerfile sources ROS 2 Jazzy and the pinned
`/opt/autoware` installation before running:

```bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The overlay build runs packages sequentially with one compiler job. This keeps
peak memory usage safe on the 16 GB target PC; the first build takes longer,
while later builds reuse Docker's layer cache.

The custom package prefixes are expected to resolve to
`/opt/selfcar_overlay` after that setup file is sourced.

To rebuild after changing a tracked package, edit only its complete package
under `vtd_overlay/src/`, run `./scripts/verify-reproducibility.sh`, and then
run `./scripts/build-vtd-image.sh` again. Do not copy generated `build/`,
`install/`, or `.so` files into the repository.

## Optional VTD bridge

The bridge source is built into the same image. Its runtime still needs the
external VTD installation:

```bash
./vtd_bridge
```

`./vtd_bridge` checks `.env`, `VTD_INSTALL_DIR`, and the locally built image,
then starts the bridge through Docker Compose. The RDB server defaults to
`VTD_RDB_HOST` from `.env`, or `127.0.0.1` when the variable is omitted. A
one-time address can be supplied without editing files:

```bash
./vtd_bridge 192.168.0.20
```

The launcher refuses to start while another VTD bridge container is running,
which prevents duplicate ROS topic and TF publishers.

The bridge container mounts only `/opt/vtd` from `VTD_INSTALL_DIR` and the X11
socket. It does not mount the bridge workspace or a host-built library.

## External assets

The following are intentionally outside Git:

| Asset | Environment variable | Expected content |
| --- | --- | --- |
| Autoware maps | `AUTOWARE_MAP_DIR` | map directories used by the base demos |
| VTD map set | `VTD_MAP_DIR` | `VTD_MAP_RELATIVE_PATH`, currently `HL_FMA_VTD_LivingLab_topology_fixed` |
| ML models | `AUTOWARE_ML_MODELS_DIR` | Autoware model artifacts |
| VTD installation | `VTD_INSTALL_DIR` | `Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h` and the VTD runtime |

The current machine's map and VTD installation locations, current file names,
and the recovered local-only inputs are recorded in
[docs/reproducibility-audit.md](reproducibility-audit.md).
