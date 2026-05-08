# Blue Robotics 수상 드론(BlueBoat) 기술 자료

> 수집일: 2026-05-05  
> 출처: [bluerobotics.com](https://bluerobotics.com), [ardupilot.org](https://ardupilot.org)

---

## 1. 개요 (Overview)

**BlueBoat**는 Blue Robotics의 무인 수상 선박(USV, Uncrewed Surface Vessel)으로, 수로 조사(hydrographic survey), 로보틱스 시스템 개발, 과학 임무 등을 목적으로 설계된 오픈 플랫폼이다.

- **특징**: 저렴한 가격 + 오픈소스(ArduRover + BlueOS) + 확장성
- **용도**: 수질 조사, 수심 측량, 해양 환경 모니터링, 자율 수면 임무
- **Hull**: 잡초/이물질에 강한 weedless 쌍동선(catamaran) 구조
- **운반**: 차 트렁크 적재 가능한 접이식 설계

---

## 2. 주요 제원 (Specifications)

### 2.1 성능 (Performance)

| 항목 | 값 |
|------|-----|
| **최대 속도** | 3 m/s (6 knots) |
| **최대 정적 추력** | 8.2 kgf (18 lbf) |
| **조향 방식** | 차동 추력 (Differential thrust) |
| **동작 온도** | -25 ~ 40°C |

### 2.2 항속 / 항속거리 (Endurance @ 1 m/s, 무탑재)

| 배터리 구성 | 용량 | 항속 시간 | 항속 거리 |
|------------|------|----------|----------|
| 2개 (기본) | 532 Wh / 2.4 kg | 18시간 | 65 km |
| 4개 | 1064 Wh / 4.7 kg | 36시간 | 130 km |
| 6개 | 1596 Wh / 7.0 kg | 50시간 | 180 km |
| 8개 | 2128 Wh / 9.3 kg | 62시간 | 220 km |
| 태양광 (별도) | — | 무제한 | — |

### 2.3 물리 제원 (Physical)

| 항목 | 값 |
|------|-----|
| **크기 (전개)** | 120 × 93 × 46 cm |
| **크기 (접힘)** | 120 × 71 × 24 cm |
| **크기 (패키지)** | 120 × 46 × 20 cm |
| **무게** (배터리/탑재물 제외) | 14.5 kg (32 lb) |
| **Freeboard** (배터리 2개, 무탑재) | 160 mm |
| **최대 흘수** (최대 탑재) | 320 mm |
| **탑재 용량** (배터리 + 탑재물) | 15 kg (33 lb) |

### 2.4 추진 시스템 (Propulsion)

| 항목 | 값 |
|------|-----|
| **모터** | M200 Motor × 2 (Weedless Propeller) |
| **프로펠러 직경** | 112 mm (4.4 in) |
| **ESC** | Basic ESC 500 (BlueBoat 전용 구성) |
| **추천 입력 전압** | 4S Li-ion (16 V) |
| **최대 입력** | 6S (단, 모터당 500W 초과 금지) |

#### M200 Motor 특징
- Flooded 3상 브러시리스 모터
- 스테이터는 수분 절연 캡슐 처리
- 자성체 로터 = 스테인리스 코팅 (해수 부식 저항)
- 수냉식 (물이 모터를 통과하며 냉각)
- 플라스틱 부싱으로 수중 윤활

### 2.5 재질 (Materials)

- 선체(Hull): LDPE
- 구조물: 30% 유리섬유 강화 폴리카보네이트, 아노다이징 알루미늄, 스테인리스 스틸

---

## 3. 전자 / 제어 시스템 (Electronics & Control)

### 3.1 컴퓨팅

| 항목          | 사양                                       |
| ----------- | ---------------------------------------- |
| **온보드 컴퓨터** | Raspberry Pi 4 (2GB) + BlueOS            |
| **비행 컨트롤러** | Navigator Flight Controller              |
| **GPS**     | mRobotics M10034-M9N (NEO-M9N + IST8308) |

### 3.2 Navigator 센서 구성

- 6-DOF IMU
- 듀얼 3-DOF 나침반
- 내부 기압계
- GPS (NEO-M9N)

### 3.3 전원 관리

| 항목 | 사양 |
|------|------|
| 직접 연결 (배터리 전압) | 최대 60 A |
| Fuse Board 경유 | 최대 10 A |
| 5V 보조 전원 | 5 A |
| **파워 스위치** | PowerSwitch (120A, 4S/6S 배터리, 전류/전압 감지) |

### 3.4 포트 / 인터페이스

- 3× Serial UART
- Ethernet (Blue Robotics Ethernet Switch 필요)
- 2× USB 2.0 / 2× USB 3.0
- 1× 16-bit ADC (3.3 V)
- 1× 16-bit ADC (6.6 V)
- 침투 홀(Penetrator): Starboard 2×M10, Port 4×M10 + 2×M14

---

## 4. 통신 시스템 (Communications)

| 항목                     | 사양                                |
| ---------------------- | --------------------------------- |
| **무선 프로토콜**            | 802.11a/b/g/n (2.4 GHz)           |
| **라우터**                | MikroTik RBGroove-52HPn           |
| **기본 동작 모드**           | Client Mode (CPE)                 |
| **BaseStation IP**     | 192.168.2.3                       |
| **BlueBoat IP**        | 192.168.2.4                       |
| **BlueOS 기본 주소**       | blueos.local (192.168.2.2)        |
| **기본 안테나 이득**          | 7 dBi 2.4GHz 전방향, N-type          |
| **무선 통신 거리 (기본 안테나)**  | 최대 250 m                          |
| **무선 통신 거리 (지향성 안테나)** | 800 m 이상                          |
| **셀룰러/위성**             | 가능 (4G LTE, Iridium, Starlink 호환) |

---

## 5. 소프트웨어 스택 (Software Stack)

### 5.1 ArduRover (차량 제어 펌웨어)

- 지상 차량 및 수상 선박을 위한 오픈소스 자율 주행 펌웨어
- ArduPilot 프로젝트 기반
- 수상/지상/범선/이족 보행 등 다양한 플랫폼 지원

**지원 자율 주행 기능:**

| 기능 | 설명 |
|------|------|
| Waypoint Navigation | GPS 기반 경유지 자동 항법 |
| Position Hold (Loiter) | 특정 위치 고정 |
| Click-to-Navigate (Guided) | 클릭 기반 즉시 이동 |
| Follow-Me | 이동 중인 대상 추종 |
| Geo-Fencing | 허용 구역 이탈 방지 |
| Return-to-Home (RTH) | 신호 단절 시 자동 귀환 |
| Manual Control | 조이스틱 수동 제어 |

### 5.2 BlueOS (온보드 운영 시스템)

- Raspberry Pi 4 기반 임베디드 OS
- Wi-Fi Access Point 내장 → 브라우저에서 접속 가능
- 확장 기능: Extension 시스템으로 3rd party 소프트웨어 설치 가능
- 포트 설정, 로그, 파라미터 관리 UI 제공

### 5.3 지상 제어 소프트웨어

- **QGroundControl** (추천)
- **Mission Planner**

---

## 6. 탑재물 통합 (Payload Integration)

### 6.1 설계 철학

두 선체 사이의 공간을 **100% 탑재물 공간**으로 활용. 제어 전자장치·배터리·배선은 모두 선체 내부에 수납.

- 선체 내벽이 평탄 → **저항파(low-wake) 센서 장착 구역** 형성
- 크로스바, 브래킷, 선체 하단에 다양한 마운트 포인트

### 6.2 지원 탑재물 예시

| 분류 | 종류 |
|------|------|
| 음향 측심기 | Single-beam / Multibeam echosounder |
| 음향 센서 | Side Scan Sonar, Imaging Sonar |
| 수질 센서 | 수온, pH, DO 등 Sonde 계열 |
| 영상 센서 | 하향식 카메라 |
| 항법 보조 | RTK GPS, USBL, AHRS |
| 레이저 | 레이저 스캐너(Lidar) 가능 |

### 6.3 RTK GPS 통합 예시

```
GPS_Type       = NMEA
Serial2_Protocol = GPS
Serial 2 설정: udpin:0.0.0.0:27000
```

NMEA 포지션 스트림을 수신하는 모든 장치와 호환 가능.

---

## 7. 자율 주행 연구 동향 (2024-2025 Trends)

### 7.1 MIT Marine Autonomy Lab

2024년 여름부터 MIT Marine Autonomy Lab + MIT Lincoln Laboratory 공동으로 **MOOS-IvP** 자율화 스택을 BlueOS에 통합하는 작업 진행 중. BlueOS Extension을 통해 배포 예정. NMEA 문자열 기반으로 타 자율화 스택과의 연동도 가능.

참고: [MIT Oceanai BlueBoat 프로젝트](https://oceanai.mit.edu/pavlab/pmwiki/pmwiki.php?n=Proj.Blueboat)

### 7.2 함대 운용 (Fleet Operation)

여러 대의 BlueBoat을 **단일 BaseStation**에 연결하여 다중 선박 동시 운용 가능.

### 7.3 위성 통신

- ArduPilot의 위성 통신 지원 (Iridium 모뎀: 짧은 메시지 / 제한된 피드백)
- **Starlink** 연동 시 완전 원격 제어 가능 (인터넷 연결 기반)

---

## 8. 관련 부품 (Key Components)

### 8.1 T200 Thruster

- BlueROV2 등에 사용되는 범용 수중 추력기
- 동작 전압: 7 ~ 20 V
- Flooded 모터 특허 설계
- BlueBoat에는 M200 Motor (노즐 제거 버전) 사용

### 8.2 Navigator Flight Controller

- Raspberry Pi HAT 형태의 비행 컨트롤러
- ArduPilot 펌웨어 실행
- 6-DOF IMU, 듀얼 나침반, 기압계 내장
- BlueOS와 직접 통합

### 8.3 Basic ESC 500

- BlueBoat 전용 설정 ESC
- 500W 출력 지원
- 4S/6S 배터리 호환

---

## 9. 우리 프로젝트 적용 포인트 (수면 부유물 수집 드론)

### 9.1 하드웨어 구성 제안

```
BlueBoat (기본 플랫폼)
├── Navigator Flight Controller (ArduRover)
├── GPS (기본 포함, RTK 확장 가능)
├── 라이다 (2D LiDAR → 장애물 회피 + 부유물 탐지)
│   └── 예: RPLidar A1/A2, Hokuyo URG-04LX
├── 초음파 센서 (근거리 장애물, 수심 측정)
│   └── 예: Blue Robotics Ping Sonar, US-100
└── 수집 메커니즘 (크로스바 마운트 활용)
```

### 9.2 자율 주행 파이프라인

```
센서 데이터 수집 (LiDAR + 초음파)
    ↓
부유물 탐지 (영상/포인트클라우드 분석)
    ↓
경로 계획 (ArduRover Waypoint / Guided Mode)
    ↓
차동 추력 제어 (M200 Motor × 2)
    ↓
수집 완료 후 RTH (Return-to-Home)
```

### 9.3 개발 스택 권장

| 레이어      | 기술                                 |
| -------- | ---------------------------------- |
| 자율 항법 FW | ArduRover (C++)                    |
| 온보드 OS   | BlueOS (Python extension 가능)       |
| 고수준 자율화  | ROS2 / MOOS-IvP (BlueOS Extension) |
| GCS      | QGroundControl                     |
| 통신       | MAVLink (UDP/Serial)               |

---

## 10. 참고 링크 (References)

| 항목                        | URL                                                                                         |
| ------------------------- | ------------------------------------------------------------------------------------------- |
| BlueBoat 제품 페이지           | https://bluerobotics.com/store/boat/blueboat/blueboat/                                      |
| BlueBoat 데이터시트 (JAN 2025) | https://bluerobotics.com/wp-content/uploads/2023/03/BLUEBOAT-DATASHEET-v1.1-JAN-2025.pdf    |
| M200 Motor                | https://bluerobotics.com/store/thrusters/t100-t200-thrusters/m200-motor/                    |
| T200 Thruster             | https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/           |
| ArduRover 문서              | https://ardupilot.org/rover/                                                                |
| BlueOS GitHub             | https://github.com/bluerobotics/BlueOS                                                      |
| Community Forum           | https://discuss.bluerobotics.com/                                                           |
| MIT BlueBoat 프로젝트         | https://oceanai.mit.edu/pavlab/pmwiki/pmwiki.php?n=Proj.Blueboat                            |
| BlueBoat ArduPilot 발표     | https://discuss.ardupilot.org/t/blueboat-released-blue-robotics-rises-to-the-surface/108864 |

---

*마지막 업데이트: 2026-05-05*
