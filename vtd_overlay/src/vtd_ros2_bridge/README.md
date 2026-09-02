# VTD–ROS 2–Autoware bridge

VTD 2025.2의 RDB 데이터를 ROS 2/Autoware 토픽으로 변환하고, Autoware의 종·횡방향 제어 명령을 VTD dynamics 입력으로 되돌려 보내는 브릿지입니다.

## 데이터 흐름

| 방향 | VTD / ROS 입력 | 브릿지 출력 |
|---|---|---|
| VTD → ROS | `RDB_OBJECT_STATE` extended | `/localization/kinematic_state`, `/localization/acceleration`, `/vehicle/status/velocity_status`, TF `map→base_link`, `/clock` |
| VTD → Autoware | 첫 ego pose 수신 이후 | `/localization/initialization_state = INITIALIZED` |
| VTD bridge → Autoware | VTD ground-truth 주행 모드 | `/perception/occupancy_grid_map/map` (ego 주변 free-space grid) |
| VTD bridge → Autoware | VTD ground-truth 주행 모드 | `/perception/obstacle_segmentation/pointcloud` (empty obstacle cloud) |
| VTD → ROS | `RDB_VEHICLE_SYSTEMS`, `RDB_DRIVETRAIN` | steering/gear/light/control-mode vehicle status |
| VTD → ROS | `RDB_RAY` | `sensor_msgs/PointCloud2` |
| VTD → ROS | `RDB_IMAGE`, `RDB_CAMERA` | `sensor_msgs/Image`, `CameraInfo` |
| VTD → Autoware | `RDB_TRAFFIC_LIGHT` extended | `/simulator/input/traffic_signals` → `/perception/traffic_light_recognition/traffic_signals` |
| Autoware planning → RViz | 현재 도로의 맵 최고속도 | `/planning/scenario_planning/applied_velocity_limit` (표시 전용) |
| VTD SHM → ROS | OptiXLidar `CUSTOM_OPTIX_START` + `RGBA32F` | `sensor_msgs/PointCloud2` |
| Autoware → VTD | control/gear/turn-indicator/hazard command topics | `RDB_DRIVER_CTRL` (`accelTgt`, `steeringTgt`, `gear`, `flags`) |

인터페이스 API의 상태 필드는 다음 ROS 토픽으로도 그대로 제공됩니다.

| 인터페이스 API | ROS 토픽 | 메시지 |
|---|---|---|
| `egoX/Y/Z`, `egoHeading/Pitch/Roll` | `/vtd/ego_state` | `vtd_ros2_bridge/msg/VtdEgoState` |
| `objects[].id/x/y/z/heading/speed/length/width/height` | `/vtd/objects` | `vtd_ros2_bridge/msg/VtdObjectArray` |
| `trafficLights[].id/state` | `/vtd/traffic_lights` | `vtd_ros2_bridge/msg/VtdTrafficLightArray` |
| `steering`, `targetAccel`, `turnSignal` | `/control/command/*` | Autoware control and turn-indicator messages |

`Object(Array=30)`의 상한은 브리지에서 30개로 적용합니다. VTD 지도에서는 RDB traffic-light
ID 하나가 신호등 전체가 아니라 빨강·노랑·초록 램프 하나를 나타냅니다. 브리지는
OpenDRIVE `signal_type`으로 램프의 색을, RDB `stateMask`로 실제 점등 여부를 판정합니다.
매핑되지 않은 신호에 대해서만 정규화된 `state`와 extended phase 정보를 fallback으로
사용합니다.

Autoware 표준 출력은 `autoware_perception_msgs/msg/TrafficLightGroupArray`입니다. VTD/OpenDRIVE
signal ID를 현재 Lanelet2 지도의 traffic-light regulatory-element ID로 바꾸는 매핑은
`config/traffic_light_id_map.csv`에 있습니다. 지도 재생성 후에는 다음처럼 매핑도
다시 생성합니다(CRDesigner 환경 필요).

매핑에는 OpenDRIVE의 `signal_type`과 `signal_subtype`도 함께 저장합니다. 지도 변환기가
멀리 떨어진 신호를 regulatory element에 연결한 경우가 있으므로, 생성기는 전체 맵에서
정지선 위치·차로 진행방향·OpenDRIVE controller가 모두 일치하는 신호만 연결합니다.
신호가 실제로 없는 차로의 잘못된 regulatory element와 우회전 전용 차로의 신호 규칙은
정리된 Lanelet2 출력에서 제거합니다. 브리지는 켜진 램프만 색과 방향 화살표로 변환합니다.

RViz 속도제한 표시는 전역 외부 상한인 `current_max_velocity` 대신 behavior path에서
ego와 가장 가까운 lane ID를 찾고, 그 lane의 reference-path 최고속도를 표시용
`VelocityLimit` 메시지로 변환한 토픽을 사용합니다. 정지선이나 곡률로 낮아진 목표속도가
아니라 현재 도로의 맵 최고속도를 표시하며, planner나 controller 입력으로 되돌아가지
않습니다.

```bash
/home/a/.venvs/crdesigner/bin/python \
  scripts/generate_traffic_light_id_map.py \
  /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/HL_FMA_VTD_LivingLab_laneoffset_fixed.xodr \
  /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/lanelet2_map.osm \
  config/traffic_light_id_map.csv \
  --commonroad /home/a/vtd_autoware_maps/HL_FMA_VTD_LivingLab_topology_fixed/map.commonroad.xml \
  --cleaned-osm-output /tmp/lanelet2_map.cleaned.osm \
  --audit-report /tmp/traffic_light_cleanup_report.json
```

엑셀에 적힌 TCP 9910, UDP 9912, RTSP 8554는 외부 참가자/Host 연결 포트입니다. 현재
실행본은 VTD 로컬 RDB(48190/48185/48192, SHM)를 사용하고 ROS 토픽으로 변환합니다.
엑셀에는 해당 포트의 패킷 직렬화 형식이 정의되어 있지 않으므로, 그 바이너리 네트워크
프로토콜을 임의로 추가하지 않았습니다.

`accelTgt`와 Autoware acceleration은 모두 m/s²이고, `steeringTgt`와 Autoware `steering_tire_angle`은 모두 앞바퀴 등가 조향각(rad)입니다. 따라서 steering wheel ratio나 pedal percentage로 바꾸지 않습니다.

## 빌드와 실행

VTD와 Autoware가 호스트에서 실행 중일 때 다음 한 줄로 빌드 후 브릿지를 실행할 수 있습니다.

```bash
cd /home/a/vtd_ros2_bridge_ws
./src/vtd_ros2_bridge/scripts/run_bridge_docker.sh
```

이 스크립트는 현재 설치된 Autoware Jazzy 이미지, host network, host IPC를 사용합니다. 직접 실행할 경우에는 다음 환경이 필요합니다.

```bash
export VTD_ROOT=/home/a/VIRES/VTD.2025.2
source /opt/ros/jazzy/setup.bash
source /opt/autoware/setup.bash
cd /home/a/vtd_ros2_bridge_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch vtd_ros2_bridge vtd_bridge.launch.py
```

다른 VTD 호스트를 쓰려면 launch 인자로 바꿉니다.

```bash
./src/vtd_ros2_bridge/scripts/run_bridge_docker.sh rdb_host:=192.168.0.20
```

## VTD 설정

1. 현재 프로젝트의 ModuleManager 설정에 아래 raw RDB TCP 포트가 있어야 합니다. VTD 기본 포트는 `48190`입니다.

   ```xml
   <RDB>
       <Port name="RDBraw" number="48190" type="TCP" />
   </RDB>
   ```

2. ego state는 extended `RDB_OBJECT_STATE`가 포함되어야 합니다. extended가 아니면 위치는 나오지만 속도와 가속도는 0으로 게시됩니다.
3. 카메라 TCP 스트림은 VTD 기본 image port `48192`를 사용합니다. 해당 IG 설정이 TCP image stream을 만들지 않는 경우에는 image port를 `0`으로 끄고 SHM을 사용합니다.
4. PerfectSensor 같은 별도 RDB sensor module은 프로젝트에서 정한 포트로 연결합니다. 현재 `SampleProject.vpj`의 `Sensor_MM`은 TCP `48185`이고, 일반 예제 ModuleManager 설정은 `48195`를 쓰기도 합니다. 표준 `RDB_RAY`가 포함될 때 PointCloud2로 변환됩니다.
5. OptiXLidar/RoboSense 계열이 System V SHM을 쓰면 `config/vtd_bridge.param.yaml`의 `shm.key`를 설정합니다. 이 설치본에서 확인된 일반 IG key는 `33130 (0x816a)`, RoboSense 샘플 기본값은 `33162`입니다. 실제 값은 VTD IG 설정과 `ipcs -m`으로 확인해야 합니다.

사용하지 않는 채널은 port를 `0`으로 설정하면 재접속 시도를 하지 않습니다.

## 좌표 동기화

VTD inertial과 ROS는 모두 x-forward/y-left/z-up인 우수 좌표계입니다. 현재 Lanelet2 경로는 2-D 기준으로 사용하므로 `flatten_z: true`가 기본이며, ego TF/odometry와 VTD 객체 pose의 z를 0으로 발행합니다. 객체 높이는 `Shape.dimensions.z`와 합성 obstacle pointcloud에 그대로 보존됩니다.

```yaml
map_offset:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
flatten_z: true
```

VTD 시나리오 원점과 Lanelet2 원점이 다르면 2-D rigid transform만 보정합니다.

```text
p_autoware = R(map_offset.yaw) · p_vtd + [map_offset.x, map_offset.y, map_offset.z]
```

좌표를 맞출 때는 VTD와 RViz에서 동일한 두 지점의 `(x,y)`를 얻어 yaw를 먼저 계산하고 translation을 계산합니다. y축 부호를 임의로 뒤집으면 좌·우 차선과 조향 부호가 함께 깨지므로 reflection 파라미터는 제공하지 않습니다.

브릿지가 `map→base_link`를 게시하므로 다른 localization 노드가 같은 TF를 동시에 게시하면 안 됩니다. VTD ground truth를 localization으로 쓸 때는 기존 NDT/localization TF publisher를 끄거나 `publish_map_to_base_tf: false`로 설정해야 합니다.

## Autoware 연결 시 주의점

- 이 노드는 `/vehicle/status/control_mode`를 `AUTONOMOUS`로 합성합니다. 실제 VTD 운전자 모드 상태가 아니라 Autoware vehicle interface 계약을 만족시키기 위한 값입니다.
- 첫 Control 메시지 전에는 가속·조향을 덮어쓰지 않고 기본 기어 `D`만 VTD에 전송합니다. 첫 Control 이후 명령이 `control_timeout_sec` 동안 끊기면 `watchdog_deceleration`을 보냅니다.
- 기본 설정은 VTD가 RDB 기어 변경을 적용하는 동안에도 Autoware가 승인한 기어 명령을 `/vehicle/status/gear_status`에 즉시 반영합니다. 실제 VTD drivetrain 피드백만 표시하려면 `report_commanded_gear: false`로 바꿉니다.
- 좌/우 방향지시등과 비상등 명령은 각각 `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`에서 받아 VTD RDB driver flags로 전송합니다. 수락 상태는 `/vehicle/status/turn_indicators_status`, `/vehicle/status/hazard_lights_status`로 게시합니다.
- Autoware가 미교전 또는 명령 종료를 나타낼 때 보내는 `NO_COMMAND`는 VTD에서 직전 램프 상태가 남지 않도록 `DISABLE`로 정규화합니다.
- 기본 `report_commanded_lights: true`는 VTD 램프의 실제 점멸 주기에 따라 status가 매번 DISABLE로 흔들리지 않도록 수락된 명령 상태를 유지합니다. VTD `RDB_VEHICLE_SYSTEMS.lightMask`를 직접 상태로 쓰려면 `false`로 설정합니다.
- `scenario_simulation:=true`에서는 Autoware occupancy-grid 노드가 비활성화되므로, 브릿지가 ego 주변의 빈 점유 격자를 발행해 behavior path planner의 입력 계약을 만족시킵니다. 실제 센서 기반 점유 격자를 연결할 때는 `publish_empty_occupancy_grid: false`로 바꿉니다.
- 같은 모드에서는 VTD 객체 바운딩박스의 모서리를 합성 obstacle pointcloud로 발행합니다. 객체가 없을 때만 빈 cloud가 되며, VTD LiDAR/실제 perception 출력을 별도로 연결하면 `publish_empty_obstacle_pointcloud: false`로 바꿉니다.
- 현재 시나리오는 `<Description ... Control="external" Name="Ego"/>`이므로 기본값은 `ego_player_id: -1`, `ego_name: Ego`입니다. 브릿지가 첫 object-state에서 실제 player ID를 찾은 후 그 ID로 제어합니다.
- Autoware의 steering tire rotation rate는 VTD의 `steeringSpeed`(steering-wheel 계열 값)와 의미가 같다고 보장되지 않아 보내지 않습니다. 앞바퀴 target angle만 사용합니다.
- `/clock`은 RDB 패키지 수가 아니라 `state/control`의 고유 frame마다 한 번만 게시합니다. `send_control_every_frame: true`에서도 조향·램프 subscriber callback이 같은 frame에 패킷을 추가 전송하지 않으므로 VTD 제어는 frame당 한 번입니다.
- VTD에서 `Stop` 후 `Start`해 raw simulation time/frame이 0으로 돌아가도 ROS 시간은 뒤로 돌리지 않습니다. 첫 세션은 system time에 고정하고 이후 세션에는 연속 offset을 적용하며, reset 시 ego 선택·차량 상태·제어/센서 캐시를 비웁니다. 따라서 기존 TF 캐시가 새 데이터를 `TF_OLD_DATA`로 버리지 않아 Autoware 재시작 없이 다시 사용할 수 있습니다.
- `/home/a/autoware_run`은 `scenario_simulation:=true`, `localization_sim_mode:=none`으로 실행되어 `simple_planning_simulator`를 띄우지 않습니다. 따라서 VTD 브릿지가 `/localization/kinematic_state`와 `map→base_link`의 유일한 발행자입니다.
- `/simulator/input/traffic_signals`은 Autoware의 dummy traffic-light passthrough를 거쳐 계획 모듈이 사용하는 `/perception/traffic_light_recognition/traffic_signals`로 전달됩니다.
- 브릿지와 Autoware 컨테이너는 모두 host network/IPC와 `/home/a/autoware/docker/files/cyclonedds.xml`을 사용합니다. 이 설정을 빼면 서로의 토픽이 보이지 않을 수 있습니다.
- `TurnIndicatorsCommand`의 좌/우와 `HazardLightsCommand`의 비상등은 RDB `flags`로 변환되며, 비상등이 좌/우 명령보다 우선합니다.
- `/vtd/objects`와 `/vtd/traffic_lights`는 RDB 상태 채널에 해당 패키지가 포함될 때 갱신됩니다. 현재 시나리오에 해당 객체가 없으면 배열은 빈 상태가 정상입니다.
- LiDAR SHM의 OptiX 기본 출력은 world 좌표의 XYZ와 packed intensity이므로 PointCloud2 frame은 `map`입니다. `RDB_RAY`는 패키지의 coordinate type에 따라 `map`, `base_link`, `lidar_link`를 선택합니다.
- RoboSense 전용 플러그인이 거리/각도를 vendor packet 형식으로만 구성한 경우에는 이 generic XYZ 변환 대신 해당 모델의 UDP/packet decoder를 추가해야 합니다.

## 빠른 확인

```bash
ros2 topic hz /localization/kinematic_state
ros2 topic echo /simulator/input/traffic_signals --once
ros2 topic echo /perception/traffic_light_recognition/traffic_signals --once
ros2 topic echo /vehicle/status/velocity_status --once
ros2 topic echo /vehicle/status/turn_indicators_status --once
ros2 topic echo /vehicle/status/hazard_lights_status --once
ros2 topic hz /sensing/lidar/top/pointcloud_raw
ros2 topic hz /sensing/camera/camera0/image_raw
ros2 topic echo /diagnostics --once
```

연결은 되었는데 ego update가 0이면 가장 먼저 `ego_player_id`와 VTD raw RDB의 extended object-state 출력을 확인합니다. 진단 정보에는 각 채널 연결 상태, ego/sensor frame 수, 제어 송신 수, parser error 수가 포함됩니다.
