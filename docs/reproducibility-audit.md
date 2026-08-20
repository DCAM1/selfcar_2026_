# Reproducibility audit

Audit date: 2026-08-20 (Asia/Seoul).

The historical runtime was captured from
`docker/examples/basic/dev-nvidia.compose.yaml` at root commit `8d332b0`.
The source workspaces were inspected before migration without changing them:

- `/home/a/autoware/src/universe/autoware_universe` was clean at
  `b10300980e9009e61721808de13dbc27b414f1a3`.
- `/home/a/autoware/src/launcher/autoware_launch` was clean at
  `66b0a248159f783bed2c6d5d3770fc49c0ba05e6`.
- Both nested repositories contain local commits authored as `Autoware Local
  Backup`; their upstream bases are recorded in `vtd_overlay/origin.yaml`.
- `/home/a/autoware_vtd_overlay/src` contained seven empty package
  directories. Its `build/` and `install/` metadata proved the package names,
  target libraries and CMake source roots, but it did not contain recoverable
  source. The source was recovered from the clean nested workspace instead.
- `/home/a/autoware/src.zip` is a 945 MiB, 17,848-entry untracked source
  snapshot. It is ignored and was not used as the authoritative source because
  it predates the current nested-repository commits.

Classification: `A` is already tracked in the root repository, `B` was local
custom source/configuration, `C` is an upstream-identical file, `D` is a build
artifact, and `E` is an allowed external runtime asset.

## Historical compose and runtime inputs

The `host path` and `realpath` columns describe the machine on which the
historical compose was observed. `Git tracking` describes the state before this
migration; nested-repository files were not tracked by the root repository.

| Reference before migration | Host path | realpath | Exists | Git tracking | Class | SHA-256 | Origin / upstream commit | Upstream difference | Final location or removal | Verification |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| compose:18 X11 socket | `/tmp/.X11-unix` | `/tmp/.X11-unix` | yes | external | E | not applicable | host display | not source | remains an explicit X11 runtime mount | compose config |
| compose:19 Autoware maps | `/home/a/autoware_data/maps` | same | yes | external | E | directory; see asset manifest | external map data | not source | `${AUTOWARE_MAP_DIR}` → `/home/aw/autoware_data/maps:ro` | script path check |
| compose:20 ML models | `/home/a/autoware_data/ml_models` | same | yes | external | E | empty on audit machine | external model data | not source | `${AUTOWARE_ML_MODELS_DIR}` → `/home/aw/autoware_data/ml_models:ro` | script path check |
| compose:21 VTD map set | `/home/a/vtd_autoware_maps` | same | yes | external | E | directory; key files below | external VTD map data | not source | `${VTD_MAP_DIR}` → `/home/aw/vtd_autoware_maps:ro` | selected map check |
| compose:22 CycloneDDS | `/home/a/autoware/docker/files/cyclonedds.xml` | same | yes | A | `1e5d541135bd9c07853ab0d419c9c2d632114570ebe882d9935110ab13017c9c` | root Autoware parent `10718787ba6e28f038a0cb29ff99cc627b5abfd2`, local commit `e9f4dd0` | yes | `docker/files/cyclonedds.xml`, copied into image | Dockerfile install |
| compose:23 vehicle info | `/home/a/autoware/src/launcher/autoware_launch/vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml` | same | yes | B in nested repo | `450c78963674b037cf80b0d571d28021ad6e3dd0ce02c80537f2bab6ff4bf22f` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7`; local head `66b0a248...` | yes | `config/vtd/vehicle/vehicle_info.param.yaml` | Dockerfile install |
| compose:24 planner `.so` | `/home/a/autoware_vtd_overlay/install/autoware_behavior_path_planner/lib/libautoware_behavior_path_planner_lib.so` | same | yes | no | D | `5985c3f25708d5025e610440ec60cd633660f108ffcb10a16387dd42640a8fb2` | target name and CMake cache identify `autoware_behavior_path_planner`; source recovered from `autoware_universe@b1030098...` | source differs from `0.52.0` | `.so` excluded; complete package in `vtd_overlay/src/.../autoware_behavior_path_planner` | colcon/Docker rebuild |
| compose:25 planner-common `.so` | `/home/a/autoware_vtd_overlay/install/autoware_behavior_path_planner_common/lib/libautoware_behavior_path_planner_common.so` | same | yes | no | D | `ced05c77035d2c4d1e52def2f163e1c15159147178333fd857357e2ff4827ece` | `autoware_universe@b1030098...` | source differs from `0.52.0` | `.so` excluded; complete package in `vtd_overlay/src/.../autoware_behavior_path_planner_common` | colcon/Docker rebuild |
| compose:26 lane-change `.so` | `/home/a/autoware_vtd_overlay/install/autoware_behavior_path_lane_change_module/lib/libautoware_behavior_path_lane_change_module.so` | same | yes | no | D | `154da9945cb4761104f3fb2106bd3cf2128a89c90ba083dab339f420d08d3c58` | `autoware_universe@b1030098...` | source differs from `0.52.0` | `.so` excluded; complete package in `vtd_overlay/src/.../autoware_behavior_path_lane_change_module` | colcon/Docker rebuild |
| compose:27 static-avoidance `.so` | `/home/a/autoware_vtd_overlay/install/autoware_behavior_path_static_obstacle_avoidance_module/lib/libautoware_behavior_path_static_obstacle_avoidance_module.so` | same | yes | no | D | `bef055de5eccc6e04d7e8b1165e3915646cfbcbfe46bad751726b53c70303e15` | `autoware_universe@b1030098...` | source differs from `0.52.0` | `.so` excluded; complete package in `vtd_overlay/src/.../autoware_behavior_path_static_obstacle_avoidance_module` | colcon/Docker rebuild |
| compose:28 mission planner `.so` | `/home/a/autoware_vtd_overlay/install/autoware_mission_planner_universe/lib/libautoware_mission_planner_universe_lanelet2_plugins.so` | same | yes | no | D | `968214f2b9106b40a0f3947894b19a3653db0be1808e2e80ff9331b229fd1ca3` | `autoware_universe@b1030098...` | source differs from `0.52.0` | `.so` excluded; complete package in `vtd_overlay/src/.../autoware_mission_planner_universe` | colcon/Docker rebuild |
| compose:29 traffic-light launch config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml` | same | yes | B in nested repo | `051c97402c341b20a93c77b8a310cd3d58c7f7c250486fb951c02433dc48ff0e` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/planning/.../behavior_velocity_planner/traffic_light.param.yaml` | Dockerfile install |
| compose:30 AD API config | `/home/a/autoware/docker/files/default_adapi_vtd.param.yaml` | same | yes | A | `04c716790882da4200c72a4b0fe97df85c0fd291ebde7e991f360fd9039d3ffd` | local root commit `e9f4dd0`; no upstream file | local-only | remains at `docker/files/default_adapi_vtd.param.yaml`, copied into image | Dockerfile install |
| compose:31 RViz | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz` | same | yes | B in nested repo | `cb0ed0965032e30a6666164c5c8c979462ba2788e2791b6653ad9edb930eee39` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/rviz/autoware.rviz` | Dockerfile install |
| compose:32 common planning config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml` | same | yes | B in nested repo | `a089a7c7099840c9043d048cd48a9967b4ffd133dfbad66f96893b51ab74d51d` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/planning/.../common/common.param.yaml` | Dockerfile install |
| compose:33 velocity smoother | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml` | same | yes | B in nested repo | `f7e417b55db6986b8ebef6c0d724d3166ee460d06a683d12a7b228ea99672de6` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/planning/.../autoware_velocity_smoother/velocity_smoother.param.yaml` | Dockerfile install |
| compose:34 drivable-area config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml` | same | yes | B in nested repo | `3bace7d182439e52104236008e702a3afab527a9d01acbe677056c51f2b938a3` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/planning/.../drivable_area_expansion.param.yaml` | Dockerfile install |
| compose:35 lane-change config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml` | same | yes | B in nested repo | `be23f838b2afe00b445e258bd6af2b6a24aa2278ca223eee8e5348dc22d0ad3d` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/planning/.../lane_change/lane_change.param.yaml` | Dockerfile install |
| compose:36 AEB config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/control/autoware_autonomous_emergency_braking/autonomous_emergency_braking.param.yaml` | same | yes | B in nested repo | `2b7dfcb5ab228d017fe0d3f84f5a7bb282d3b876986245abdc55256af377064b` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/control/autonomous_emergency_braking/autonomous_emergency_braking.param.yaml` | Dockerfile install |
| compose:37 PID config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml` | same | yes | B in nested repo | `43766d4e40257bd3b6db6d56cfb8d3f2f5def26a2f0db81ece2ddbc1219fc388` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/control/trajectory_follower/longitudinal/pid.param.yaml` | Dockerfile install |
| compose:38 MPC config | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml` | same | yes | B in nested repo | `3f775316211c219950b9ce29db8132776162e3f420d2029b0b3a3d328effd902` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/control/trajectory_follower/lateral/mpc.param.yaml` | Dockerfile install |
| compose:39 control diagnostics | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/control.yaml` | same | yes | B in nested repo | `92c6f88253f1d5dd0c25c6e3bd44e04f23c597e87eae69e102bf59f8948fa33f` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/diagnostics/control.yaml` | Dockerfile install |
| compose:40 localization diagnostics | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/localization.yaml` | same | yes | B in nested repo | `57c12306a292d651861a6a5ebbcea8a3ec13f78023331d279e40cfda5eb4caaa` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/diagnostics/localization.yaml` | Dockerfile install |
| compose:41 system diagnostics | `/home/a/autoware/src/launcher/autoware_launch/autoware_launch/config/system/diagnostics/system.yaml` | same | yes | B in nested repo | `1eeaed50bcae9b96273ed6319103cf7219848e0f59f3af465847a3865562445e` | `autoware_launch@f942598d44b5769353167c76b784323d5c14c8c7` | yes | `config/vtd/diagnostics/system.yaml` | Dockerfile install |
| historical overlay path | `/home/a/autoware_vtd_overlay/src/*` | same | dirs only; files absent | no | B evidence source absent | n/a | build metadata only; CMake source root `/workspace/src/...` | source not recoverable there | no copy; recovered from nested Git instead | audit blocker resolved by nested source |
| overlay path-optimizer `.so` | `/home/a/autoware_vtd_overlay/install/autoware_path_optimizer/lib/libautoware_path_optimizer.so` | same | yes | no | D/C | `dff5be8e464587f74ce79eb74e8d57f9a3e908ddf8b2557c28ac8ff7d3eb59ec` | package source compared identical to current upstream workspace | no custom diff from `0.52.0` established | not copied; binary remains excluded | source comparison |

## Bridge and VTD runtime inputs

The running bridge container `recursing_mclean` separately mounted the bridge
workspace and VTD installation. The bridge source is now built into the VTD
image; VTD itself remains an external prerequisite.

| Reference | Host path | realpath | Exists | Git tracking | Class | SHA-256 | Origin / upstream commit | Final location | Verification |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| bridge container workspace | `/home/a/vtd_ros2_bridge_ws` | same | yes | no | B | source tree `d124187bd42471183b346c6e2992642e49e1e2bc7200b26e3ffef68658786285` after the tracked CMake portability fix | local-only ROS package; no upstream remote | `vtd_overlay/src/vtd_ros2_bridge` | package structure and Docker build |
| bridge container VTD mount | `/home/a/VIRES/VTD.2025.2` | same | yes | external | E | `viRDBIcd.h` `1b68ce86142929a1e12bf2c0570d622ba616152ac65af3306dd589f969742d58`; `viRDBTypes.h` `5b6b146f6a502ded4545155854998824c04d3529b846d45fce98069d995b199b` | VIRES VTD 2025.2 installation | `${VTD_INSTALL_DIR}` build named context and optional runtime mount `/opt/vtd` | build script header check |
| bridge CycloneDDS mount | `/home/a/autoware/docker/files/cyclonedds.xml` | same | yes | A | E/A | see row above | root tracked configuration | image layer `/home/aw/cyclonedds.xml` | Dockerfile install |

## External asset manifest

These assets are intentionally not committed. Directory hashes are not used as
the acquisition identity; the listed key files provide useful integrity checks.

| Asset | Current location | Expected file/name | SHA-256 | Acquisition/use |
| --- | --- | --- | --- | --- |
| VTD map set | `/home/a/vtd_autoware_maps` | `HL_FMA_VTD_LivingLab_topology_fixed/lanelet2_map.osm` | `7f0edab75f51f7640feb272d468979b192a348aa685a903c9e5b8582ee472cb8` | Set `VTD_MAP_DIR`; obtain the VTD/Lanelet2 map separately |
| VTD map metadata | same | `HL_FMA_VTD_LivingLab_topology_fixed/map_projector_info.yaml` | `a6fb6af33e164e61a075f9a79eeffa5eca3ead4435dc3de49e811525175b8307` | Required by the planning simulator |
| VTD OpenDRIVE input | same | `HL_FMA_VTD_LivingLab_topology_fixed/HL_FMA_VTD_LivingLab_laneoffset_fixed.xodr` | `5d2f45f7bf33418b5233a634ec45ef12a096470b6438bf5d87a770c6b39979b9` | Used by the VTD map workflow |
| Autoware maps | `/home/a/autoware_data/maps` | map directories such as `sample-map-planning` | directory hash not calculated | Set `AUTOWARE_MAP_DIR`; obtain via the Autoware artifact instructions |
| ML models | `/home/a/autoware_data/ml_models` | model files | empty at audit time | Set `AUTOWARE_ML_MODELS_DIR`; obtain the required model artifacts separately |
| VTD installation | `/home/a/VIRES/VTD.2025.2` | `Develop/Communication/VtdApi/lib_cxx11/include/VtdToolkit/viRDBIcd.h`; `Develop/Communication/Common/viRDBTypes.h` | `viRDBIcd.h` `1b68ce86142929a1e12bf2c0570d622ba616152ac65af3306dd589f969742d58`; `viRDBTypes.h` `5b6b146f6a502ded4545155854998824c04d3529b846d45fce98069d995b199b` | Set `VTD_INSTALL_DIR`; install VTD 2025.2 separately |

## Result

The five historical `.so` mounts now have complete source-package
replacements. The sixth overlay library (`autoware_path_optimizer`) was an
artifact whose source matched the upstream baseline and was deliberately not
duplicated. All custom launch/config inputs used by the historical compose are
tracked under `config/vtd` or the existing tracked `docker/files` paths.

## Rebuilt library comparison

The old five mounted libraries were compared with the corresponding libraries
from the rebuilt image using `nm -D --defined-only | c++filt | sort -u` after
removing the address/type columns. The old CMake metadata showed `Release`; the
Dockerfile uses the same build type. All five exported symbol-name sets were
identical (planner 8,100, planner-common 1,582, lane-change 981,
static-obstacle 1,049, mission-planner 1,144). Byte-for-byte equality was not
used as a requirement.
