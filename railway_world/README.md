# Gazebo Korean Railway Synthetic Environment

열차 없이 선로·승강장·전차선·건널목·주변 시설을 구성한 Gazebo Sim 8 / Harmonic용 합성 데이터 월드입니다. 특정 역의 as-built 복제본이 아니라, 한국 일반철도 규격과 제공된 승강장 사진을 결합한 reference-grade 환경입니다.

## 구성

- 정확히 500 m인 표준궤 본선: 승강장 뒤 252 m까지 직선, 이후 반경 400 m의 완만한 좌곡선
- 50N 근사 실단면 레일, 1:40 근사 내향 경사, 600 mm 간격 PC 침목, 레일패드·e-clip형 체결구
- 녹슨 레일 측면과 연마된 주행면·게이지면, 3D 쇄석 입자가 겹쳐진 PBR 자갈 도상
- 120 m 측식 승강장: 회색 콘크리트, 황색 점자블록, 긴 금속 캐노피, 조명
- 벤치, 유리 방풍벽, 역명판 형식 표지, LED 안내판, 휴지통
- 48 m 전철주 경간, 레일면 위 5.2 m 전차선, 960 mm 가고, 처짐 조가선과 드로퍼
- 동쪽 432 m 지점의 도로 건널목: 고무 보판, 차단기, 경보기, 노면 표시
- 330 m 지점 우분기 분기기와 약 68 m 곡선 측선: 포인트 레일, 가드레일, 전철기, 차막이
- 펜스, 케이블 트로프, 관리도로, 배수로, 설비함, 유지보수 건물, 식생
- 월드 고정 카메라 없음; 향후 드론 모델에 RGB·depth·semantic·instance 센서를 탑재하도록 분리

## 실행

```bash
cd railway_world
./run.sh
```

`run.sh`는 카메라 센서 시스템만 제외한 임시 GUI 월드를 실행합니다. 침목·체결구·레일·3D 쇄석은 중첩 모델 대신 최적화된 단일 모델 내부 형상으로 표시되어 디테일과 로딩 성능을 함께 유지합니다.

드론 연동용 서버 실행:

```bash
./run_synthetic.sh
```

PX4 X500과 하방 RGB 카메라를 함께 실행:

```bash
./run_x500.sh
```

X500은 월드에 영구 포함되어 Entity Tree에 `x500`으로 표시됩니다. 기본 위치는 승강장 앞 본선 중앙 `(-118, 0, 0.60)`이며, 시작 화면도 이 위치를 바라봅니다. `run_x500.sh`는 새 기체를 생성하지 않고 이 엔티티에 PX4 SITL을 연결합니다. 하방 영상 토픽은 `/x500/rail_down_camera/image`입니다. 종료할 때 실행 터미널에서 `Ctrl+C`를 누르면 PX4와 Gazebo가 함께 종료됩니다.

## 지원 및 검증 환경

이 프로젝트는 다음 조합에서 작성·검증했습니다.

| 항목 | 버전/조건 |
|---|---|
| 운영체제 | Ubuntu 22.04 LTS (Jammy), x86_64 |
| Gazebo | Gazebo Sim 8 / Harmonic (`gz sim --versions` 첫 줄이 `8.x`) |
| Python | Python 3.10 |
| PX4 | release/1.15 계열, 기준 커밋 `85df8c2281c2466b30a121b22b0bf33dc69bcfe4` |
| 그래픽 | OpenGL 지원 GPU와 정상 동작하는 드라이버 권장 |

Ubuntu 24.04도 Gazebo Harmonic 공식 바이너리가 제공되지만, 이 저장소의 기준 환경은 Ubuntu 22.04입니다. Windows 네이티브와 macOS는 이 프로젝트의 실행 스크립트로 검증하지 않았습니다. 공식 자료는 [Gazebo Harmonic Ubuntu 설치 안내](https://gazebosim.org/docs/harmonic/install_ubuntu/)와 [PX4 Ubuntu 개발 환경 안내](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)를 참고하세요.

## 다른 컴퓨터에 설치

### 1. 기본 월드만 실행

PX4 비행이 필요 없고 Gazebo 월드만 확인할 때 사용하는 최소 설치입니다.

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg git python3-pip python3-venv

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic

git clone https://github.com/LEE-YOONSU/offboard_rail_following_drone.git
cd offboard_rail_following_drone/railway_world
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
./check_environment.sh
./run.sh
```

가상환경에 `--system-site-packages`를 사용하는 이유는 `gz.transport13`과 `gz.msgs10` Python 모듈이 Gazebo의 Ubuntu 시스템 패키지로 설치되기 때문입니다. GUI 없이 서버만 실행하려면 `./run_synthetic.sh`를 사용합니다.

### 2. PX4 X500 비행까지 실행

PX4 공식 설치 스크립트가 빌드 도구와 Gazebo 시뮬레이션 의존성을 설치합니다. 아래 명령은 이 프로젝트를 만들 때 사용한 PX4 커밋으로 고정하므로 다른 컴퓨터에서도 동작 차이를 줄일 수 있습니다.

```bash
cd "$HOME"
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout 85df8c2281c2466b30a121b22b0bf33dc69bcfe4
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh --no-nuttx
```

설치 스크립트가 끝나면 컴퓨터를 재부팅하고 X500 SITL을 한 번 빌드합니다.

```bash
cd "$HOME/PX4-Autopilot"
make px4_sitl gz_x500
```

기본 PX4/Gazebo 창이 뜨면 `Ctrl+C`로 종료한 뒤 이 프로젝트를 실행합니다.

```bash
cd /path/to/offboard_rail_following_drone/railway_world
source .venv/bin/activate
./check_environment.sh
./run_x500.sh
```

`run_x500.sh`는 월드에 이미 들어 있는 `x500` 엔티티에 PX4 SITL을 standalone 모드로 연결합니다. PX4를 다른 위치에 설치했다면 다음과 같이 경로를 지정합니다.

```bash
PX4_DIR=/path/to/PX4-Autopilot ./run_x500.sh
```

PX4와 Gazebo 연동 방식 및 환경변수는 [PX4 Gazebo Simulation 안내](https://docs.px4.io/main/en/sim_gazebo_gz/)에서 확인할 수 있습니다.

### 3. 설치 확인 및 문제 해결

```bash
./check_environment.sh
gz sim --versions
```

- `Expected Gazebo Sim 8` 오류: Gazebo Harmonic이 아닌 다른 주 버전이 우선 실행되고 있습니다.
- `PX4 SITL binary not found`: `PX4_DIR`가 올바른지 확인하고 PX4 폴더에서 `make px4_sitl gz_x500`을 실행합니다.
- Wayland에서 GUI가 뜨지 않음: 실행 스크립트가 Wayland 환경에서는 Qt XCB 백엔드를 자동 선택하므로 `xwayland` 설치 여부를 확인합니다.
- VM 또는 원격 데스크톱에서 렌더링 문제: GPU 가속을 켜거나 PX4 실행 전에 `export PX4_GZ_SIM_RENDER_ENGINE=ogre`를 시도합니다.
- 다른 Gazebo가 실행 중이라는 오류: 기존 `gz sim` 프로세스를 정상 종료한 뒤 `./run_x500.sh`를 다시 실행합니다.

## 데이터셋과 Git 관리

`datasets/`와 `output/`은 생성 결과라서 Git 본문에는 포함되지 않습니다. 데이터셋은 [`datasets-20260820` GitHub Release](https://github.com/LEE-YOONSU/offboard_rail_following_drone/releases/tag/datasets-20260820)에서 내려받습니다.

저장소를 복제하고 `railway_world` 폴더로 이동한 뒤 다음 명령을 실행합니다.

```bash
curl -LO https://github.com/LEE-YOONSU/offboard_rail_following_drone/releases/download/datasets-20260820/railway_world_datasets_20260820.tar.gz
curl -LO https://github.com/LEE-YOONSU/offboard_rail_following_drone/releases/download/datasets-20260820/railway_world_datasets_20260820.tar.gz.sha256
sha256sum -c railway_world_datasets_20260820.tar.gz.sha256
tar -xzf railway_world_datasets_20260820.tar.gz
```

검증 결과가 `OK`이면 현재 폴더 아래에 `datasets/`가 생성됩니다. 압축 파일은 606,260,464바이트이며 SHA-256은 `e9c5c083cb15f2f85468d2638fa534ed8d518fa155f67f1d3fb1003ca6722a17`입니다.

공개 저장소로 만들기 전에 `metadata/`와 문서에 외부 공개가 곤란한 현장 정보가 없는지 확인하세요.

## 드론 센서 연결

현재 월드에는 고정 카메라가 없으므로 `/synthetic/*` 영상 토픽도 생성되지 않습니다. 클래스 라벨은 `config/labels.yaml`에 유지됩니다. 향후 드론 모델에 센서를 붙일 때 `models/synthetic_sensor_rig/model.sdf`와 `config/capture.yaml`을 탑재용 템플릿으로 사용할 수 있습니다.

## 주요 파일

- 전체 배치: `worlds/railway_environment.sdf`
- 플랫폼: `models/station_platform_12m/model.sdf`
- 건널목: `models/level_crossing/model.sdf`
- 분기기·곡선 측선: `models/railway_turnout/model.sdf`
- 곡선용 24 m 전차선 경간: `models/catenary_span_24m/model.sdf`
- 드론 탑재용 센서 템플릿(현재 미사용): `models/synthetic_sensor_rig/model.sdf`
- PX4 X500 하방 카메라 모델: `models/x500_rail_down_cam/model.sdf`
- 클래스 라벨: `config/labels.yaml`
- 기준값과 출처: `docs/reference_basis.md`
- 현장 기준 파라미터: `metadata/site_reference.yaml`
- 시설물 자산대장: `metadata/asset_register.csv`

서쪽 끝이 chainage 0 m, 동쪽 곡선 끝이 500 m입니다. 승강장 구간은 촬영과 플랫폼 정합을 위해 직선으로 유지했습니다. 실제 디지털 트윈으로 전환하려면 측량 좌표, 선형, LiDAR/사진측량 자료와 실제 시설물 자산대장을 연결해야 합니다.
