# Offboard Rail-Following Drone Experiments

PX4 기반 철도 추종 드론을 서로 다른 시험 환경에서 검증하기 위한 저장소입니다. 기존 비정형 도로 시험과 새 Gazebo 철도 합성 환경을 독립된 폴더로 관리합니다.

## 폴더 구성

| 폴더 | 내용 |
|---|---|
| [`unstructured_road_test/`](./unstructured_road_test/) | ROS 2 Humble, MAVROS, YOLO 및 depth obstacle avoidance를 이용한 기존 비정형 도로 시험 코드 |
| [`railway_world/`](./railway_world/) | Gazebo Harmonic용 500 m 한국형 철도 월드, X500 하방 카메라 모델, 합성 데이터 캡처 도구 |

각 환경의 설치 조건과 실행 명령은 해당 폴더의 README를 참고하세요.

## 내려받기

```bash
git clone https://github.com/LEE-YOONSU/offboard_rail_following_drone.git
cd offboard_rail_following_drone
```

철도 환경만 실행하려면:

```bash
cd railway_world
./check_environment.sh
./run.sh
```

철도 환경의 생성 데이터셋은 [`datasets-20260820` Release](https://github.com/LEE-YOONSU/offboard_rail_following_drone/releases/tag/datasets-20260820)에서 내려받을 수 있으며, 렌더 출력은 Git에서 제외되어 있습니다.
