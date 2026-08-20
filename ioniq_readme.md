# IONIQ 6 VTD Vehicle Model for Autoware

> VTD 2025.2.438에서 `HyundaiIoniq6_23.xml` 차량에 실제 적용되는 `SimpleVehicleModel`을 분석하고,  
> 동일한 거동을 Autoware 기반 프로젝트에서 재현하기 위한 구현 가이드입니다.

---

## 1. 목적

이 문서의 목적은 VTD에서 사용 중인 IONIQ 6 차량 모델을 Autoware 환경에서 최대한 동일하게 재현하는 것입니다.

현재 VTD의 IONIQ 6는 고정밀 타이어 기반 차량 동역학 모델이 아닙니다.

활성화된 모델은 `SimpleVehicleModel`이라는 이산 single-track 계열 모델이며, 다음 요소는 포함되지 않습니다.

- 타이어 slip ratio 상태
- 타이어 slip angle 상태
- 노면 마찰계수 `mu` 입력
- 전/후륜 타이어 force
- cornering stiffness
- yaw inertia
- load transfer
- friction circle
- suspension force의 평면 차량 거동 피드백
- 실제 타이어 포화 기반 understeer / oversteer

고속에서 나타나는 understeer는 실제 타이어 slip으로 계산되는 것이 아니라 다음 `selfSteering` 항으로 인위적으로 만들어집니다.

```text
K_ss = 0.0105
```

따라서 **VTD의 현재 차량 거동을 그대로 Autoware에서 재현하려는 목적이라면 Pacejka나 고차 dynamic bicycle model을 사용하는 것보다 아래 이산 모델을 직접 구현하는 것이 더 적합합니다.**

---

# 2. VTD에서 실제 활성화된 차량 모델

현재 VTD setup에서는 다음 두 플러그인이 핵심적으로 동작합니다.

## 2.1 HLVTD

외부 제어 명령을 VTD의 `RDB_DRIVER_CTRL`로 전달하는 bridge 역할을 합니다.

## 2.2 viTrafficDyn

차량의 다음 상태를 계산하고 적분합니다.

- 위치
- 속도
- 가속도
- heading
- steering
- yaw rate
- pitch / roll 표현값
- wheel rotation 등

내부 생성 시 다음 설정이 사용됩니다.

```text
modelType = 0
```

이는 `SimpleVehicleModel`을 의미합니다.

Bullet 기반의 `TrafficDynComplex`는 현재 setup에서 활성화되어 있지 않습니다.

또한 다음 파일은 기본적으로 시각 모델입니다.

```text
HyundaiIoniq6_23.ive
```

따라서 이 파일 안에 고정밀 타이어 slip, suspension force, tire friction 등의 차량 동역학 정보가 포함되어 있다고 가정하면 안 됩니다.

---

# 3. Autoware에서 구현할 모델의 상태와 입력

VTD의 `accelTgt`, `steeringTgt`를 직접 전달하는 모드를 기준으로 최소 상태 벡터를 다음과 같이 두는 것이 좋습니다.

\[
x_k =
\begin{bmatrix}
X_k &
Y_k &
\psi_k &
v_k &
a_k &
\delta_k
\end{bmatrix}^{T}
\]

각 상태는 다음 의미를 갖습니다.

| 변수 | 의미 | 단위 |
|---|---|---|
| `X` | world X position | m |
| `Y` | world Y position | m |
| `psi` | yaw / heading | rad |
| `v` | longitudinal velocity | m/s |
| `a` | longitudinal acceleration | m/s² |
| `delta` | steering angle | rad |

입력은 다음과 같습니다.

\[
u_k =
\begin{bmatrix}
a_{\mathrm{cmd},k} &
\delta_{\mathrm{cmd},k}
\end{bmatrix}^{T}
\]

| 입력 | 의미 | 단위 |
|---|---|---|
| `a_cmd` | target longitudinal acceleration | m/s² |
| `delta_cmd` | target steering angle | rad |

추가 외생 입력은 다음과 같습니다.

- 실제 simulation timestep `h`
- OpenDRIVE에서 얻은 road pitch `theta`
- forward / reverse / neutral 방향

> 주의: VTD timestep은 동역학 내부 고정값이 아닙니다.  
> 따라서 Autoware 모델에서도 `1/60 s` 같은 상수를 가정하지 말고 실제 주기 `dt`를 사용해야 합니다.

---

# 4. IONIQ 6 주요 차량 파라미터

VTD 차량 XML 및 활성 모델에서 사용되는 값은 다음과 같습니다.

| Parameter | Symbol | Value |
|---|---:|---:|
| Mass | `m` | 2095 kg |
| Engine Power | `P` | 240000 W |
| Overall Efficiency | `eta` | 0.75 |
| Air Drag Coefficient | `Cd` | 0.28 |
| Effective Frontal Area | `A` | 2.3 m² |
| Rolling Resistance | `Crr` | 0.013 |
| XML Max Speed | `v_max` | 70 m/s |
| Max Deceleration | `a_brake_max` | -9.5 m/s² |
| Wheelbase | `L` | 2.944 m |
| Max Steering | `delta_max` | 0.48 rad |
| Self Steering | `K_ss` | 0.0105 |
| Steering Slew Rate | `delta_rate_max` | 1.047 rad/s |
| Air Density | `rho` | 1.202 kg/m³ |
| Gravity | `g` | 9.80665 m/s² |
| Wheel Drive |  | FWD |

---

# 5. 종방향 모델

## 5.1 공기저항

공기저항 계수는

\[
q = \frac{1}{2}\rho C_d A
\]

이며 IONIQ 6 파라미터를 대입하면

\[
q = 0.387044
\]

입니다.

평지 기준 저항 가속도는

\[
a_{\mathrm{res}}(v)
=
-\frac{qv^2}{m}
-C_{rr}g
\]

이며 수치화하면

\[
a_{\mathrm{res}}(v)
=
-0.00018474654v^2
-0.12748645
\]

입니다.

즉 throttle을 주지 않아도 drag와 rolling resistance에 의해 차량은 자연 감속합니다.

---

## 5.2 Road Pitch

VTD 내부 구현은 다음 형태입니다.

\[
a_{\mathrm{res}}
=
g\theta_{\mathrm{VTD}}
-\frac{qv^2}{m}
-C_{rr}g
\]

일반적인 물리 모델에서 자주 사용하는

\[
g\sin(\theta)
\]

가 아니라 pitch 값 자체를 사용한다는 점에 주의해야 합니다.

또한 pitch 부호는 VTD 좌표계 정의와 실제 OpenDRIVE road orientation에 따라 확인이 필요합니다.

Autoware 구현 시에는 실제 ramp scenario에서 다음을 검증해야 합니다.

```text
+theta -> uphill ?
+theta -> downhill ?
```

VTD와 동일한 방향으로 맞추는 것이 목적입니다.

---

# 6. Power Limit

출력으로 인한 최대 가속도는

\[
a_{\mathrm{power}}
=
\frac{P\eta}
{m\max(|v|,\epsilon)}
\]

입니다.

IONIQ 6 값을 대입하면

\[
a_{\mathrm{power}}
=
\frac{85.9188544}
{\max(|v|,\epsilon)}
\]

입니다.

속도가 증가할수록 동일한 power에서 낼 수 있는 acceleration은 감소합니다.

---

# 7. Traction Limit

현재 차량은 FWD입니다.

VTD 내부 traction acceleration cap은 다음과 같습니다.

| Drive Type | Acceleration Cap |
|---|---:|
| FWD | 4.8 m/s² |
| RWD | 5.5 m/s² |
| AWD | 9.0 m/s² |

IONIQ 6에서는

\[
a_{\mathrm{traction}}
=
4.8\cos(\theta)
\]

를 사용합니다.

중요한 점은 VTD 내부 함수 이름이 `get_slip_accel` 계열이라도 실제 tire slip을 계산하지 않는다는 것입니다.

실제 구현은 구동 방식에 따라 위 고정 acceleration limit을 반환하는 수준입니다.

---

# 8. 실제 가능한 구동 가속도

구동 가능한 acceleration은

\[
a_{\mathrm{drive}}
=
\min
\left(
a_{\mathrm{traction}},
a_{\mathrm{power}}
\right)
\]

입니다.

저항까지 포함하면

\[
a_{\mathrm{potential}}
=
a_{\mathrm{drive}}
+
a_{\mathrm{res}}
\]

입니다.

IONIQ 6에서는 traction limit과 power limit의 교차 속도가 약

\[
v \approx 17.90\ \mathrm{m/s}
\]

입니다.

이는 약

```text
64.4 km/h
```

입니다.

따라서 대략적으로 다음 구조입니다.

```text
저속
  ↓
FWD traction cap 지배
  ↓
약 64 km/h
  ↓
240 kW power limit 지배
```

---

# 9. 최고속도와 Taper

Power와 주행 저항이 평형을 이루는 속도는 다음 방정식으로 계산됩니다.

\[
qv^3+C_{rr}mgv=\eta P
\]

양의 근은 약

```text
74.51 m/s
```

입니다.

그러나 XML MaxSpeed가

```text
70 m/s
```

이므로 최종 속도 제한은 70 m/s입니다.

약 252 km/h입니다.

또한

\[
0.9v_{\max}=63\ \mathrm{m/s}
\]

부터 다음 taper가 적용됩니다.

\[
T(v)=
\begin{cases}
1,&v\le0.9v_{\max}\\
\max\left(0,10\frac{v_{\max}-v}{v_{\max}}\right),
&v>0.9v_{\max}
\end{cases}
\]

그리고

\[
a_{\mathrm{potential}}
\leftarrow
T(v)a_{\mathrm{potential}}
\]

로 처리됩니다.

---

# 10. Acceleration Command 처리

외부에서 전달되는 `accelTgt`는 먼저 다음 범위로 clamp됩니다.

\[
-20
\le
a_{\mathrm{cmd}}
\le
20
\]

일반적인 acceleration branch에서는

\[
a_{k+1}
=
\min
\left(
a_{\mathrm{cmd}},
a_{\mathrm{potential}}
\right)
\]

입니다.

예를 들어 controller가

```text
a_cmd = 10.0 m/s²
```

를 보내더라도 현재 차량 상태에서 가능한 acceleration이

```text
4.5 m/s²
```

라면 실제 사용되는 값은 약 4.5 m/s²가 됩니다.

---

# 11. 강제 제동 Branch

강제 제동 branch 조건은

\[
a_{\mathrm{cmd}}-a_{\mathrm{res}}
<
-0.3a_{\mathrm{power}}
\]

입니다.

조건이 참이면

\[
a_{k+1}
=
\max
\left(
a_{\mathrm{cmd}},
a_{\mathrm{res}}-9.5\cos\theta,
-9.5
\right)
\]

형태로 처리됩니다.

극저속에서는 `a_power`가 매우 커질 수 있으므로 branch 선택이 다소 비물리적으로 보일 수 있습니다.

이 부분은 실제 차량 물리라기보다는 VTD 내부 heuristic으로 보는 것이 적절합니다.

---

# 12. 속도 적분

VTD는 단순 Forward Euler

\[
v_{k+1}=v_k+a_kh
\]

를 사용하지 않습니다.

속도 변화량은

\[
\Delta v
=
\frac{a_k+a_{k+1}}{2}h
\]

입니다.

전진 상태에서는

\[
v_{k+1}
=
\max
\left(
0,
v_k+\Delta v
\right)
\]

를 사용합니다.

---

# 13. 종방향 이동거리 적분

한 timestep 동안의 종방향 이동거리는

\[
s_{\mathrm{long}}
=
v_kh
+
\frac{2a_k+a_{k+1}}{6}h^2
\]

입니다.

따라서 단순한

```text
s = v * dt
```

구현을 사용하면 VTD와 미세한 차이가 발생할 수 있습니다.

---

# 14. Steering 모델

최대 steering angle은

\[
|\delta|_{\max}=0.48\ \mathrm{rad}
\]

입니다.

약

```text
27.5 deg
```

입니다.

먼저 steering target을 clamp합니다.

\[
\delta_{\mathrm{target}}
=
\operatorname{clamp}
(
\delta_{\mathrm{cmd}},
-0.48,
0.48
)
\]

이후 steering slew-rate limit을 적용합니다.

\[
\delta_{k+1}
=
\delta_k+
\operatorname{clamp}
\left(
\delta_{\mathrm{target}}-\delta_k,
-1.047h,
1.047h
\right)
\]

즉 최대 steering angle 변화율은

```text
1.047 rad/s
```

약 60 deg/s입니다.

---

# 15. Yaw Rate 모델

현재 VTD 횡방향 거동의 핵심 식입니다.

\[
r_{k+1}
=
\frac
{v_{k+1}\delta_{k+1}}
{L+K_{\mathrm{ss}}v_{k+1}^2}
\]

IONIQ 6에서는

```text
L    = 2.944
K_ss = 0.0105
```

입니다.

곡률은

\[
\kappa
=
\frac{r}{v}
=
\frac{\delta}
{L+K_{\mathrm{ss}}v^2}
\]

가 됩니다.

---

# 16. Understeer 구현 방식

현재 설정에서는

\[
K_{\mathrm{ss}}>0
\]

이므로 understeer 형태입니다.

일반적으로 다음처럼 볼 수 있습니다.

| `K_ss` | 특성 |
|---:|---|
| `> 0` | Understeer |
| `= 0` | Neutral steer |
| `< 0` | 수학적 Oversteer 형태 |

현재 값은

```text
K_ss = 0.0105
```

이므로 understeer만 포함되어 있습니다.

속도가 증가하면

\[
K_{\mathrm{ss}}v^2
\]

항이 커지고 denominator가 증가합니다.

따라서 같은 steering angle에서도 curvature가 감소합니다.

```text
같은 steering delta

저속
  -> 높은 curvature

고속
  -> 낮은 curvature
  -> 차량이 덜 회전
```

이것이 현재 VTD 모델의 understeer 표현입니다.

---

# 17. Characteristic Speed

self-steering 효과가 wheelbase와 비슷한 수준이 되는 특성 속도는

\[
v_{\mathrm{char}}
=
\sqrt{\frac{L}{K_{\mathrm{ss}}}}
\]

입니다.

IONIQ 6에서는

\[
v_{\mathrm{char}}
=
16.74\ \mathrm{m/s}
\]

약

```text
60.3 km/h
```

입니다.

이는 60 km/h 이상에서 갑자기 understeer가 활성화된다는 뜻이 아닙니다.

속도가 증가하면서 효과가 연속적으로 커지고, 약 60 km/h 전후부터 그 영향이 눈에 띄기 시작한다는 의미입니다.

---

# 18. 일반 Bicycle Model과의 차이

일반적인 kinematic bicycle model에서는 자주 다음 식을 사용합니다.

\[
r
=
\frac{v}{L}\tan(\delta)
\]

하지만 VTD SimpleVehicleModel은

\[
r
=
\frac{v\delta}
{L+K_{\mathrm{ss}}v^2}
\]

형태입니다.

즉 다음 두 차이가 있습니다.

1. `tan(delta)`가 아니라 `delta` 자체를 사용
2. `K_ss * v^2`를 분모에 추가하여 고속 steering response 감소

따라서 Autoware에서 기본 bicycle model을 그대로 사용하면 VTD와 동일하지 않습니다.

---

# 19. 횡가속도와 Heading

횡가속도는

\[
a_{y,k+1}
=
v_{k+1}r_{k+1}
\]

입니다.

Heading은

\[
\psi_{k+1}
=
\operatorname{fmod}
(
\psi_k+r_{k+1}h,
2\pi
)
\]

입니다.

---

# 20. 횡속도와 횡방향 이동거리

횡속도는

\[
v_{y,k+1}
=
v_{k+1}\tan(r_{k+1}h)
\]

로 매 step 새로 생성됩니다.

그리고

\[
s_{\mathrm{lat}}
=
v_{y,k+1}h
\]

입니다.

이 `v_y`는 실제 tire sideslip 상태가 아닙니다.

즉 이전 timestep의 lateral dynamics state를 적분해 만든 값이 아니라 현재 step의 회전량으로부터 합성한 값입니다.

---

# 21. World Position 갱신

VTD는 갱신된 heading을 사용하여 다음 위치를 계산합니다.

\[
\Delta X
=
s_{\mathrm{long}}\cos\psi_{k+1}
-s_{\mathrm{lat}}\sin\psi_{k+1}
\]

\[
\Delta Y
=
s_{\mathrm{lat}}\cos\psi_{k+1}
+s_{\mathrm{long}}\sin\psi_{k+1}
\]

최종 위치는

\[
X_{k+1}
=
X_k+\Delta X
\]

\[
Y_{k+1}
=
Y_k+\Delta Y
\]

입니다.

> VTD와 최대한 동일한 재현이 목적이라면 `psi_k`가 아니라 갱신된 `psi_{k+1}`를 사용해야 합니다.

---

# 22. 한 Step 전체 State Update

Autoware에서 동일 모델을 구현할 경우 전체 순서는 다음과 같이 구성하는 것이 좋습니다.

```text
Input:
  a_cmd
  delta_cmd
  dt
  road_pitch

        │
        ├───────────────────────────────┐
        │                               │
        ▼                               ▼
Longitudinal Model                 Steering Model
        │                               │
        ▼                               ▼
Resistance                        Steering Clamp
Power Limit                            │
Traction Limit                         ▼
Brake Logic                       Slew Rate Limit
        │                               │
        ▼                               ▼
     a[k+1]                         delta[k+1]
        │                               │
        ▼                               ▼
Velocity Integration               Yaw Rate
        │                               │
        ▼                               ▼
     v[k+1]                         r[k+1]
        │                               │
        └──────────────┬────────────────┘
                       ▼
                 psi[k+1]
                       │
            ┌──────────┴──────────┐
            ▼                     ▼
        s_long                  s_lat
            │                     │
            └──────────┬──────────┘
                       ▼
                  X[k+1], Y[k+1]
```

---

# 23. 타이어 마찰 모델이 없다는 점

VTD RDB protocol에는 다음 필드가 존재합니다.

- wheel slip
- wheel longitudinal force
- wheel lateral force
- contact force
- contact friction
- environment friction scale
- road water level

하지만 현재 `SimpleVehicleModel`에서는 이 값들이 차량 궤적 계산에 사용되지 않습니다.

wheel slip 값도 실제 계산 결과가 아니라 사실상 `0.0`으로 기록됩니다.

따라서

```text
wheel slip = 0
```

을

```text
실제로 slip이 발생하지 않았다
```

라고 해석하면 안 됩니다.

보다 정확하게는

```text
SimpleVehicleModel은 wheel slip을 계산하지 않는다
```

라고 보는 것이 맞습니다.

---

# 24. 고속 횡가속도가 비물리적으로 커질 수 있음

최대 steering

\[
\delta=0.48\ \mathrm{rad}
\]

에서 계산되는 횡가속도는 대략 다음과 같습니다.

| Speed | Lateral Acceleration |
|---:|---:|
| 10 m/s | 약 12.0 m/s² |
| 20 m/s | 약 26.9 m/s² |
| 30 m/s | 약 34.9 m/s² |
| 70 m/s | 약 43.2 m/s² |

70 m/s에서는 약

```text
4.4 g
```

까지 계산될 수 있습니다.

실제 승용차 타이어로는 비현실적인 수치입니다.

이는 모델에

\[
F_y \le \mu F_z
\]

형태의 tire friction limit이 없기 때문입니다.

따라서 이 모델의 understeer는 **실제 tire saturation이 아니라 수학적 curvature attenuation**입니다.

---

# 25. XML 파라미터 사용 여부

## 25.1 평면 궤적에 직접 사용되는 값

- `Mass`
- `MaxSpeed`
- `MaxSteering`
- `WheelBase`
- `EnginePower`
- `OverallEfficiency`
- `AirDragCoefficient`
- `FrontSurfaceEffective`
- `RollingResistance`
- `WheelDrive`
- `MaxDecel`

## 25.2 XML에는 없지만 사용되는 내부값

- `selfSteering = 0.0105`
- `air density = 1.202 kg/m³`
- `steering rate = 1.047 rad/s`

## 25.3 주로 출력 또는 시각 효과에 사용

- `WheelDiameter`
- 차체 `Dist*`
- vehicle class
- pitch / roll 관련 parameter

## 25.4 XML에 있지만 현재 평면 궤적 계산에서는 사용되지 않는 값

- `MaxAccel`
- `MaxTorque`
- `EngineType`
- `DriveChainEfficiency`
- `WheelSkewStiffness`
- `SteeringRatio`
- `TurningCircle`
- `MaxMass`
- `DistCenterOfGravity`
- `TireWidth*`
- `TrackWidth*`
- inertia 관련 값

특히 다음 항목에 주의해야 합니다.

```text
MaxAccel = 7
```

은 현재 acceleration limit으로 사용되지 않습니다.

FWD 차량에서는 내부 고정 cap

```text
4.8 m/s²
```

가 적용됩니다.

또한

```text
SteeringRatio = 16.8
```

도 `steeringTgt` 직접 입력 경로에서는 사용되지 않습니다.

`WheelSkewStiffness = 12` 역시 현재 활성 궤적 모델에서는 실제 lateral tire force 계산에 사용되지 않습니다.

---

# 26. Pitch / Roll / Suspension

VTD에는 pitch 및 roll 계산이 존재하지만 현재 평면 운동에 feedback되지 않습니다.

예를 들어 다음 값들을 계산할 수 있습니다.

- longitudinal acceleration에 따른 pitch
- lateral acceleration에 따른 roll
- vehicle corner height
- wheel compression
- wheel rotation

하지만 다음 물리 chain으로 연결되지 않습니다.

```text
Pitch / Roll
    ↓
Load Transfer
    ↓
Wheel Normal Force
    ↓
Tire Force
    ↓
Vehicle XY Motion
```

따라서 현재 모델에서 suspension은 실제 평면 차량 거동에 영향을 주는 물리 subsystem이라고 보기 어렵습니다.

---

# 27. TrafficDynComplex와 구분

VTD 설치에는 Bullet 기반 `TrafficDynComplex`도 존재합니다.

이 모델에는 다음 요소가 포함될 수 있습니다.

- raycast wheel
- suspension
- friction update
- collision physics

하지만 현재 IONIQ 6 setup에서는 다음 이유로 사용 모델로 해석하면 안 됩니다.

1. 현재 setup에서 비활성
2. IONIQ 6 XML에 Complex 전용 tire / suspension parameter 부족
3. 활성화해도 상당수 값이 VTD 내부 기본값 사용
4. 현재 Simple 모델의 거동과 직접 동일하지 않음

따라서 Autoware에서 현재 VTD 거동을 복제하려면 Complex 모델을 기준으로 구현하면 안 됩니다.

---

# 28. Autoware에서 권장하는 구현 구조

현재 목적이 **VTD와 같은 IONIQ 6 plant model을 Autoware 프로젝트에 넣는 것**이라면 다음처럼 분리하는 것이 좋습니다.

```text
autoware_ioniq6_model/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── autoware_ioniq6_model/
│       ├── vehicle_model.hpp
│       ├── longitudinal_model.hpp
│       └── lateral_model.hpp
├── src/
│   ├── vehicle_model.cpp
│   ├── longitudinal_model.cpp
│   ├── lateral_model.cpp
│   └── vehicle_model_node.cpp
├── config/
│   └── ioniq6.yaml
├── launch/
│   └── ioniq6_vehicle_model.launch.xml
└── README.md
```

---

# 29. 권장 C++ State 구조체

예시:

```cpp
struct VehicleState
{
  double x;
  double y;
  double yaw;

  double velocity;
  double acceleration;
  double steering;
};
```

입력은 다음처럼 둘 수 있습니다.

```cpp
struct VehicleInput
{
  double acceleration_cmd;
  double steering_cmd;

  double road_pitch;
  double dt;
};
```

---

# 30. Vehicle Parameters 구조체

```cpp
struct VehicleParameters
{
  double mass = 2095.0;

  double engine_power = 240000.0;
  double overall_efficiency = 0.75;

  double air_drag_coefficient = 0.28;
  double frontal_area = 2.3;
  double air_density = 1.202;

  double rolling_resistance = 0.013;
  double gravity = 9.80665;

  double wheelbase = 2.944;

  double max_speed = 70.0;
  double max_deceleration = -9.5;

  double max_steering = 0.48;
  double max_steering_rate = 1.047;

  double self_steering = 0.0105;

  double traction_accel_fwd = 4.8;
};
```

실제 프로젝트에서는 hard coding보다 YAML parameter로 분리하는 것을 권장합니다.

---

# 31. 권장 YAML

```yaml
/**:
  ros__parameters:

    vehicle:
      mass: 2095.0
      wheelbase: 2.944

      max_speed: 70.0

      max_steering: 0.48
      max_steering_rate: 1.047

      self_steering: 0.0105

    longitudinal:
      engine_power: 240000.0
      overall_efficiency: 0.75

      air_density: 1.202
      air_drag_coefficient: 0.28
      frontal_area: 2.3

      rolling_resistance: 0.013
      gravity: 9.80665

      traction_accel_fwd: 4.8
      max_deceleration: -9.5
```

---

# 32. Autoware 연동 방향

Autoware에서 이 모델은 크게 두 가지 용도로 사용할 수 있습니다.

## A. Controller Prediction Model

MPC controller 내부 prediction model로 사용

```text
Trajectory
    ↓
MPC
    ↓
Custom IONIQ 6 Prediction Model
    ↓
Steering / Acceleration Command
```

이 경우 핵심 상태는 다음으로 충분합니다.

```text
X
Y
yaw
velocity
acceleration
steering
```

장점:

- VTD plant와 MPC 내부 model mismatch 감소
- 고속 self-steering 특성 반영 가능
- steering slew-rate 반영 가능
- acceleration saturation 반영 가능

---

## B. Standalone Vehicle Simulation Node

Autoware 제어 명령을 받아 차량 상태를 직접 적분하는 node로 사용

```text
Autoware Controller
        ↓
Control Command
        ↓
ioniq6_vehicle_model_node
        ↓
Odometry / Vehicle Status
```

이 구조는 VTD 없이 controller를 단독 테스트할 때 유용합니다.

예:

- MPC tuning
- trajectory tracking test
- acceleration / steering saturation test
- unit test
- regression test

---

# 33. 추천 ROS 2 데이터 흐름

개념적으로 다음 구성이 좋습니다.

```text
/planning/scenario_planning/trajectory
                │
                ▼
         Autoware Controller
                │
                ▼
     Ackermann Control Command
                │
                ▼
       IONIQ6 Vehicle Model
          ┌─────┴─────┐
          ▼           ▼
       Odometry    Vehicle Status
          │           │
          └─────┬─────┘
                ▼
        Localization / Control
```

실제 topic명은 사용 중인 Autoware 버전과 구성에 맞게 연결해야 합니다.

---

# 34. MPC에 넣을 때 중요한 점

VTD 모델을 그대로 prediction model에 사용하려면 다음 요소를 반드시 포함하는 것이 좋습니다.

## 필수

- variable timestep `dt`
- steering angle saturation
- steering slew-rate
- `K_ss * v²` self-steering
- acceleration power limit
- FWD traction cap
- aerodynamic drag
- rolling resistance
- acceleration saturation
- max speed taper

## 가능하면 포함

- road pitch
- VTD braking branch
- forward / reverse 처리
- VTD와 동일한 integration order

## 굳이 넣지 않아도 되는 것

현재 VTD Simple 모델과 동일성을 목표로 한다면 다음은 필요하지 않습니다.

- Pacejka
- Magic Formula
- tire slip state
- cornering stiffness
- yaw inertia
- load transfer
- suspension force
- friction circle

이들을 추가하면 물리적으로는 더 현실적일 수 있지만 오히려 현재 VTD plant와 model mismatch가 커질 수 있습니다.

---

# 35. Controller용 모델과 Simulator용 모델은 분리 권장

한 클래스에 모든 기능을 몰아넣기보다는 다음과 같이 분리하는 것이 좋습니다.

```text
VehicleModelCore
    │
    ├── LongitudinalModel
    │
    └── LateralModel
```

그리고 이를

```text
MPC Prediction Model
```

과

```text
Standalone Simulation Node
```

에서 공통 사용하도록 구성합니다.

예:

```text
                  ┌── MPC Model
VehicleModelCore ─┤
                  └── Simulation Node
```

이렇게 하면 MPC와 simulator가 같은 식을 사용하므로 구현 차이로 인한 오차를 줄일 수 있습니다.

---

# 36. 추천 핵심 API

```cpp
class Ioniq6VehicleModel
{
public:
  explicit Ioniq6VehicleModel(const VehicleParameters & params);

  VehicleState update(
    const VehicleState & state,
    const VehicleInput & input);

private:
  double calculateResistance(
    double velocity,
    double road_pitch) const;

  double calculatePowerLimit(
    double velocity) const;

  double calculateTractionLimit(
    double road_pitch) const;

  double calculateAcceleration(
    const VehicleState & state,
    const VehicleInput & input) const;

  double calculateSteering(
    double steering,
    double steering_cmd,
    double dt) const;

  double calculateYawRate(
    double velocity,
    double steering) const;

  VehicleParameters params_;
};
```

---

# 37. 핵심 Update Pseudocode

```cpp
VehicleState Ioniq6VehicleModel::update(
  const VehicleState & s,
  const VehicleInput & u)
{
  VehicleState next = s;

  // 1. acceleration
  next.acceleration =
    calculateAcceleration(s, u);

  // 2. velocity
  const double dv =
    0.5 * (s.acceleration + next.acceleration) * u.dt;

  next.velocity =
    std::max(0.0, s.velocity + dv);

  // 3. longitudinal displacement
  const double s_long =
    s.velocity * u.dt +
    ((2.0 * s.acceleration + next.acceleration) / 6.0)
    * u.dt * u.dt;

  // 4. steering
  next.steering =
    calculateSteering(
      s.steering,
      u.steering_cmd,
      u.dt);

  // 5. yaw rate
  const double yaw_rate =
    next.velocity * next.steering /
    (
      params_.wheelbase +
      params_.self_steering *
      next.velocity *
      next.velocity
    );

  // 6. heading
  next.yaw =
    std::fmod(
      s.yaw +
      yaw_rate * u.dt,
      2.0 * M_PI);

  // 7. lateral velocity / displacement
  const double vy =
    next.velocity *
    std::tan(yaw_rate * u.dt);

  const double s_lat =
    vy * u.dt;

  // 8. world position
  const double dx =
    s_long * std::cos(next.yaw) -
    s_lat * std::sin(next.yaw);

  const double dy =
    s_lat * std::cos(next.yaw) +
    s_long * std::sin(next.yaw);

  next.x = s.x + dx;
  next.y = s.y + dy;

  return next;
}
```

이 코드는 전체 architecture를 보여주기 위한 pseudocode입니다.

VTD와 정확한 bit-level 동일성을 목표로 할 경우 braking branch, reverse 처리, float/double 저장 방식, max-speed taper 등을 추가로 그대로 맞춰야 합니다.

---

# 38. 구현 우선순위

추천 구현 순서는 다음과 같습니다.

## Phase 1 — Core Model

먼저 ROS와 독립된 C++ library로 구현합니다.

```text
VehicleState + VehicleInput
        ↓
Ioniq6VehicleModel::update()
        ↓
Next VehicleState
```

목표:

- unit test 가능
- ROS dependency 제거
- MPC와 simulator에서 재사용 가능

---

## Phase 2 — Longitudinal Validation

다음 test case부터 검증합니다.

### Test 1

```text
flat road
steering = 0
constant acceleration command
```

비교:

- acceleration
- velocity
- position

### Test 2

```text
flat road
full acceleration
```

비교:

- 0 ~ 17.9 m/s traction-limited 영역
- 이후 power-limited 영역

### Test 3

```text
high speed
```

비교:

- 63 m/s 이후 taper
- 70 m/s max speed

---

## Phase 3 — Steering Validation

### Test 4

```text
constant speed
step steering command
```

검증:

- steering slew-rate
- yaw rate
- heading

### Test 5

동일 steering command에서 속도만 변경합니다.

```text
10 m/s
20 m/s
30 m/s
```

비교:

\[
r =
\frac{v\delta}
{L+K_{ss}v^2}
\]

self-steering 효과가 VTD와 동일한지 확인합니다.

---

## Phase 4 — 2D Trajectory Validation

일정 steering + 일정 속도에서 원형 궤적을 비교합니다.

비교 대상:

```text
VTD X
VTD Y
VTD yaw

vs.

Custom Model X
Custom Model Y
Custom Model yaw
```

시간에 따른 error를 기록합니다.

---

## Phase 5 — Autoware Integration

Core model 검증 후 ROS 2 interface를 연결합니다.

이 순서를 권장합니다.

```text
Core Dynamics
    ↓
Standalone ROS 2 Node
    ↓
Autoware Control Command Interface
    ↓
MPC Prediction Model
```

처음부터 Autoware controller 내부에 직접 넣으면 동역학 문제와 ROS interface 문제를 동시에 디버깅해야 하므로 비효율적입니다.

---

# 39. VTD와 반드시 비교해야 하는 항목

최종 검증 시 다음 데이터를 동일 timestep으로 기록하는 것이 좋습니다.

```text
timestamp
X
Y
yaw
vx
vy
ax
ay
yaw_rate
steering
accelTgt
steeringTgt
road_pitch
```

비교 결과는 CSV로 저장하고 다음 error를 확인합니다.

\[
e_X = X_{\mathrm{model}} - X_{\mathrm{VTD}}
\]

\[
e_Y = Y_{\mathrm{model}} - Y_{\mathrm{VTD}}
\]

\[
e_{\psi}
=
\psi_{\mathrm{model}}
-
\psi_{\mathrm{VTD}}
\]

\[
e_v
=
v_{\mathrm{model}}
-
v_{\mathrm{VTD}}
\]

---

# 40. 아직 실제 VTD 데이터로 최종 확인이 필요한 부분

현재 reverse engineering 결과를 기반으로 모델 구조는 상당 부분 복원되어 있지만 다음은 실제 RDB capture를 통해 최종 확인하는 것이 좋습니다.

## 40.1 Road Pitch Sign

VTD에서

```text
positive pitch
```

가 실제로 uphill인지 downhill인지 확인

## 40.2 Module 실행 순서

`HLVTD`에서 command가 전달된 시점과 `viTrafficDyn`에서 해당 command가 적용되는 frame을 확인

## 40.3 Acceleration State Timing

`a_k`, `a_{k+1}`가 RDB output에서 어느 frame에 기록되는지 확인

## 40.4 Steering State Timing

slew-rate 적용 전 / 후 값이 어느 RDB field에 기록되는지 확인

이 부분을 확인하면 VTD와 거의 동일한 discrete implementation을 만들 수 있습니다.

---

# 41. 최종 권장 구조

SELFCAR / Autoware 프로젝트에서는 다음 구조를 권장합니다.

```text
                 AUTOWARE
                    │
                    ▼
              Planning
                    │
                    ▼
                Control
                 /   \
                /     \
               ▼       ▼
        Longitudinal   Lateral / MPC
               \       /
                \     /
                 ▼   ▼
           IONIQ6 Model Core
                 │
                 ▼
              Commands
                 │
                 ▼
                 VTD
```

MPC 내부 prediction에서도 동일한 `IONIQ6 Model Core` 식을 사용합니다.

```text
                     ┌─────────────────┐
Reference Trajectory │                 │
────────────────────►│       MPC       │
                     │                 │
                     └───────┬─────────┘
                             │
                 Prediction uses
                             │
                             ▼
                   IONIQ6 Model Core
```

이렇게 구성하면

```text
MPC Internal Model
        ≈
VTD Vehicle Plant
```

가 되어 model mismatch를 줄일 수 있습니다.

---

# 42. 핵심 결론

현재 VTD IONIQ 6 모델은 다음과 같이 이해하면 됩니다.

```text
Simple Longitudinal Dynamics
        +
Power Limit
        +
FWD Traction Acceleration Cap
        +
Air Drag
        +
Rolling Resistance
        +
Road Pitch
        +
Steering Rate Limit
        +
K_ss * v² 기반 Self-Steering
        +
Discrete Position Integration
```

즉 실제 차량의 고정밀 tire / suspension model이 아니라 **VTD에서 빠르게 차량 거동을 표현하기 위한 단순화된 discrete single-track model**입니다.

따라서 현재 VTD 차량 거동과 최대한 동일한 MPC를 만드는 것이 목적이라면

```text
Dynamic Bicycle Model
Pacejka
Magic Formula
```

를 새로 적용하기보다 다음 상태를 기반으로 한 custom discrete model을 구현하는 것이 적절합니다.

\[
\boxed{
x =
[X,Y,\psi,v,a,\delta]^T
}
\]

입력은

\[
\boxed{
u =
[a_{\mathrm{cmd}},\delta_{\mathrm{cmd}}]^T
}
\]

이며 횡방향 핵심 식은

\[
\boxed{
r =
\frac{v\delta}
{L+0.0105v^2}
}
\]

입니다.

종방향에서 핵심 가속도 limit은 대략

\[
\boxed{
a_{\mathrm{drive}}
=
\min
\left(
4.8\cos\theta,
\frac{85.9188544}{\max(|v|,\epsilon)}
\right)
}
\]

이며 여기에

- aerodynamic drag
- rolling resistance
- road pitch
- brake logic
- max-speed taper

를 추가합니다.

---

# 43. 다음 개발 단계

이 README를 기준으로 실제 구현은 다음 순서로 진행하는 것을 권장합니다.

```text
1. ROS-independent IONIQ6Model C++ class 구현
          ↓
2. Python/C++ unit test로 식 검증
          ↓
3. VTD RDB log와 step-by-step 비교
          ↓
4. ROS 2 standalone simulation node 구현
          ↓
5. Autoware Ackermann command 연결
          ↓
6. MPC prediction model에 같은 Core Model 적용
          ↓
7. VTD closed-loop trajectory tracking 검증
```

최종적으로는 **VTD와 Autoware MPC가 같은 차량 모델을 공유하도록 만드는 것**이 가장 중요한 설계 방향입니다.
