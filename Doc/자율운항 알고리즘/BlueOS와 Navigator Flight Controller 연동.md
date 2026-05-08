문서에 있는 내용 + ArduPilot/BlueOS 아키텍처 지식을 합쳐서 정리합니다.

---

## Navigator ↔ BlueOS 통신 구조

### 핵심 구조

```
BlueOS (Raspberry Pi 4, Linux)
    │
    │  MAVLink over UART (물리 연결)
    │  /dev/ttyAMA0 또는 /dev/serialX
    │
Navigator HAT (ArduRover 펌웨어)
    ├── IMU, 나침반, 기압계
    └── PWM → ESC → 모터
```

Navigator는 Pi HAT이므로 **GPIO/UART가 직접 연결**됩니다. USB가 아닌 하드와이어드 시리얼입니다.

---

### 통신 프로토콜 레이어

|레이어|프로토콜|상세|
|---|---|---|
|물리|UART (`/dev/ttyAMA0`)|Pi ↔ Navigator HAT 하드와이어드|
|메시지|**MAVLink v2**|ArduPilot 표준 바이너리 프로토콜|
|라우팅|**MAVProxy / mavlink-router**|BlueOS 내부에서 멀티플렉싱|
|외부 노출|UDP 14550 (GCS), UDP 14551 (보조)|QGroundControl/MissionPlanner 접속 포인트|

---

### BlueOS 내부 MAVLink 라우팅

```
Navigator (ArduRover)
    ↓ UART MAVLink
mavlink-router (BlueOS 서비스)
    ├── UDP 14550 → QGroundControl (GCS)
    ├── UDP 14551 → 외부 앱 (ROS2, MOOS-IvP 등)
    └── TCP 5760  → Mission Planner 호환
```

`mavlink-router`가 단일 UART 스트림을 여러 클라이언트에 브로드캐스트하는 핵심 데몬입니다.

---

### RTK GPS 연동 예시 (문서 발췌)

```
# ArduRover 파라미터
GPS_Type         = NMEA
Serial2_Protocol = GPS

# BlueOS에서 NMEA 스트림 수신
udpin:0.0.0.0:27000
```

외부 NMEA 소스(RTK 수신기 등)를 UDP로 받아 ArduRover의 Serial2로 주입하는 구조입니다.

---

### ROS2/MOOS-IvP 연동 시

```
ROS2 (BlueOS Extension)
    └── MAVROS 또는 mavlink2ros
            └── UDP 14551 → mavlink-router → Navigator
```

고수준 자율화 스택은 MAVLink UDP를 통해 ArduRover에 `GUIDED` 모드 명령(SET_POSITION_TARGET 등)을 전송합니다.

---

### 정리

- **Navigator ↔ BlueOS**: 하드와이어드 UART + MAVLink v2 (변경 불가, HAT 구조)
- **BlueOS 내부**: `mavlink-router`가 1:N 멀티플렉싱
- **외부 접속**: UDP 14550 (GCS), 추가 포트는 BlueOS 웹UI에서 설정 가능
- **센서 주입** (GPS, LiDAR 등): UDP → ArduRover Serial 파라미터로 매핑

---

문서에는 상세 내용이 없으므로 ArduPilot 아키텍처 기준으로 정확히 설명드립니다.

---

## MAVLink 모터 제어 계층 구조

결론부터: **고수준 명령(전진/회전)**을 보내면 ArduRover 내부에서 각 모터 PWM으로 변환합니다.

```
GCS / ROS2 / 자율화 스택
    │
    │  [고수준 MAVLink 명령]
    ▼
ArduRover (Navigator)
    │  내부 믹서(Mixer)
    ▼
PWM ch1: 우측 M200   PWM ch3: 좌측 M200
```

---

### MAVLink 제어 명령 3단계

#### 1단계 — RC 스타일 (수동/반자동)

```
MANUAL_CONTROL (MAVLink #69)
    ├── x: -1000 ~ +1000  (전진/후진, Throttle)
    └── r: -1000 ~ +1000  (좌/우 회전, Steering)
```

BlueBoat 차동 추력 → 내부에서 좌우 모터 PWM으로 믹싱

#### 2단계 — 속도/위치 명령 (Guided 모드)

```
SET_POSITION_TARGET_LOCAL_NED (#84)
    ├── vx, vy: 원하는 속도 벡터
    └── yaw_rate: 회전 속도

SET_POSITION_TARGET_GLOBAL_INT (#86)
    └── lat, lon, alt: GPS 목표 좌표
```

#### 3단계 — 직접 PWM (비상/테스트용)

```
RC_CHANNELS_OVERRIDE (#70)
    ├── chan1_raw: 1100~1900 μs (우측 모터)
    └── chan3_raw: 1100~1900 μs (좌측 모터)
```

믹서를 **우회**하여 각 채널에 직접 PWM 주입. 일반적으로 권장하지 않습니다.

---

### BlueBoat 내부 믹서 동작

```
입력: Throttle(T), Steering(S)

우측 모터 PWM = T + S
좌측 모터 PWM = T - S

예) 전진:     T=500, S=0   → 우500 / 좌500
    우회전:   T=0,   S=500 → 우500 / 좌-500
    전진+우:  T=300, S=200 → 우500 / 좌100
```

ArduRover의 `FRAME_TYPE=1` (차동 구동) 설정 시 자동 적용됩니다.

---

### 자율 주행 시 권장 방식

| 상황          | 명령                               | 이유                      |
| ----------- | -------------------------------- | ----------------------- |
| Waypoint 항법 | `MAV_CMD_NAV_WAYPOINT`           | ArduRover가 경로 전체 관리     |
| 실시간 유도      | `SET_POSITION_TARGET_GLOBAL_INT` | GPS 좌표 직접 지정            |
| 속도 제어       | `SET_POSITION_TARGET_LOCAL_NED`  | 로컬 NED 프레임 속도 벡터        |
| 수동 오버라이드    | `MANUAL_CONTROL`                 | Throttle/Steering 직접 제어 |
| 개별 모터 테스트   | `RC_CHANNELS_OVERRIDE`           | 디버깅 전용                  |

수면 부유물 수집 용도라면 **Guided 모드 + `SET_POSITION_TARGET_GLOBAL_INT`** 조합이 표준입니다. 부유물 위치를 GPS 좌표로 변환 후 명령을 전송하면 ArduRover가 경로 추종 + 차동 추력 믹싱을 자동 처리합니다.