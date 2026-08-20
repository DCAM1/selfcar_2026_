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
./autoware
```

`./autoware` checks that the external map/model directories and the selected
VTD map exist, enables temporary X11 access, runs the planning simulator with
the existing launch arguments, and removes the X11 access rule on exit.

## Overlay build

`scripts/build-vtd-image.sh` validates the immutable base-image digest and the
VTD RDB header, then invokes Buildx with the VTD installation as a read-only
named build context. The Dockerfile sources ROS 2 Jazzy and the pinned
`/opt/autoware` installation before running:

```bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

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
docker compose --env-file .env -f docker/vtd/compose.yaml --profile vtd run --rm vtd_bridge
```

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
