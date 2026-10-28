# OPENVLA Lego Graph Starter v2

첨부하신 UI 스타일을 기준으로 만든 OPENVLA용 스타터 템플릿입니다.

## 포함 항목
- React + Vite + TypeScript + ReactFlow 프론트
- FastAPI 그래프 실행 백엔드
- `vla_stub` 추론 서버
- `gr00t_stub` NVIDIA Isaac GR00T 호환 추론 서버
- `finetune_stub` 학습 작업 서버
- `stt_real` 음성 인식 서버
- `openvla_real` 스켈레톤 서버
- 파일 업로드 API
- Dataset / FineTune 노드가 포함된 그래프 MVP

## 프론트 노드
- Prompt
- Image Input
- Camera Sensor
- Remote Image Sensor
- Remote LiDAR Sensor
- Fused Sensor
- Robot State
- OpenVLA Infer
- NVIDIA GR00T Infer
- Dataset Upload
- Dataset Preview
- OpenVLA FineTune
- Print

## 로봇 설정 노드

**Load → FANUC M-710iC/20L Execute → 덮어쓰기**를 누르면 **로봇 설정** 노드가 포함된 그래프를 불러옵니다. 덮어쓰기는 현재 그래프 편집 내용을 선택한 템플릿으로 교체합니다. 노드만 추가하려면 왼쪽 **Nodes → COMMON → 로봇 설정**을 누르고 필요한 설정 핀을 연결합니다. 모델·인스턴스 ID·브리지 주소·로봇 prim·Collection 기억 경로를 이 노드에서 지정하고, 연결된 준비·관측·충돌 문맥·Guard·Execute가 같은 설정을 사용합니다. 기존 저장 그래프는 자동으로 재배선하지 않습니다.

| 설정 방식 | 사용법 |
| --- | --- |
| 기본 프로파일 | 기존 M-710iC/20L 구조를 사용하고 설치별 주소·prim·보정값을 지정 |
| USD 자동 읽기 | 백엔드가 접근할 수 있는 USD와 로봇 prim 지정. 사용자 정의 6축은 모델명·새 프로파일 ID·J6 기준점 오프셋도 입력 |
| 수동 프로파일 JSON | 기본 프로파일 JSON을 받아 수정하거나, 설정/프로파일 JSON 파일을 불러오기 |

설정은 Inspector의 **설정 JSON 내보내기**로 보관하고 다른 노드·실행에서 다시 불러올 수 있습니다. 예시는 [로봇 설정 파일](examples/graphs/fanuc_m710_robot_settings.json), 구조 필드는 [M-710 프로파일](frontend/public/robot_profiles/fanuc_m710ic20l.json)을 참고하세요. `prepareSettings`에는 UF/UT·IK seed·외부축·시간 설정을 입력할 수 있습니다. 설정 노드에 입력한 값이 연결된 개별 노드 값보다 우선합니다.

자동 USD 읽기 의존성은 Docker 백엔드 이미지에 기본 포함됩니다. 호스트 Python으로 실행할 때는 `python -m pip install -r backend/requirements-usd.txt`로 설치합니다. Docker에서 제외하려면 `docker compose build --build-arg INSTALL_USD=false backend`를 사용합니다. 파일과 참조 asset은 **백엔드 프로세스/컨테이너에서** 접근 가능해야 합니다. 현재 자동 입력은 USD만 지원하며 URDF 직접 읽기는 포함하지 않습니다. 자동 추출이나 프로파일 불러오기가 TCP·작업 셀 보정·물리 실행 인증을 대신하지 않습니다.

지원 범위는 기본 M-710 및 지원 가능한 직렬 6축 구조의 **프로파일 기반 계산·진단**입니다. 임의의 고정 회전·다축·다른 제조사의 제어기는 추가 어댑터가 필요합니다. 사용자 정의 모델의 실제 전송은 차단하며, 기본 M-710의 기존 `full` 정책과 물리 미검증 차단도 유지합니다. 이동 로봇·휴머노이드·다른 제조사의 실행 어댑터는 아직 이 노드의 구현 범위가 아닙니다.

## Docker 의존성 설치 오류

`RUN pip install ...` 오류가 나면 마지막 `exit code: 1` 앞의 실제 패키지 오류를 확인합니다. MediaPipe 0.10.21은 protobuf 4.x와 NumPy 1.x가 필요한 현재 구성이므로 [backend/requirements.txt](backend/requirements.txt)에 동작 확인한 protobuf·NumPy·JAX·OpenCV 버전을 고정했습니다. [Dockerfile](backend/Dockerfile)은 다운로드 재시도·시간 제한·pip 캐시와 `pip check`를 적용합니다.

```powershell
docker compose build backend
docker compose up -d
```

다른 폴더/Compose 프로젝트로 실행 중이면 해당 프로젝트와 설정 파일을 유지해 재빌드합니다. `down -v`나 볼륨 삭제는 필요하지 않습니다.

화면에 최신 노드가 없는데 이 작업 폴더의 코드에는 있는 경우, 다른 프로젝트 폴더에서 이미지를 빌드했는지 확인합니다. 현재 실행 중인 `openvla_backend`·`openvla_frontend`의 프로젝트·볼륨 설정을 유지하면서 **이 작업 폴더의 소스**로 반영하려면 다음을 실행합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\reload-workspace-docker.ps1
```

이 스크립트는 기존 컨테이너의 Compose 설정을 읽고 빌드 경로만 현재 작업 폴더로 지정합니다. 이후 화면을 Ctrl+F5로 새로고침하고 Load에서 템플릿을 불러옵니다.

9174 프론트는 최신인데 공유 9000 백엔드에 `robot_setup` 등이 없으면 **백엔드만** 갱신합니다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\reload-workspace-docker.ps1 -BackendOnly
```

`-BackendOnly`는 기존 백엔드의 Compose 프로젝트·데이터 마운트를 유지하고 `--no-deps backend`로 재빌드합니다. 9173·9174 프론트와 추론·학습 컨테이너는 재생성하지 않습니다. 기존 CORS 허용 주소를 보존하면서 `http://localhost:9174`와 `http://127.0.0.1:9174`를 추가합니다. 9000은 두 프론트가 공유하므로 갱신된 백엔드가 두 화면의 요청을 처리합니다. `/health`의 `node_types`에서 `robot_setup` 등록 여부를 확인할 수 있습니다.

## 그래프 UI와 Bundle Selected Graph

`vla_chr\VLAgraph_project`의 UI는 비교 프로젝트의 **GRAPH / RUN 툴바**, **Nodes / Graphs 탭**, **Load 창** 흐름을 사용합니다. 기존 센서·FANUC·월드모델 노드는 유지합니다.

1. **New Graph**는 빈 캔버스를 만듭니다. **Load**에서 템플릿을 고르고 **덮어쓰기** 또는 **추가하기**를 선택합니다. 추가한 그래프는 독립된 노드·연결 ID를 사용합니다. 덮어쓰기나 새 그래프 생성 전에 필요한 편집 내용은 **Save**로 저장하세요.
2. 왼쪽 **Graphs** 탭에서 대상 그래프를 선택한 뒤 **Bundle Selected Graph**를 누릅니다. 해당 그래프 전체가 카드 하나로 접힙니다. 캔버스의 개별 노드 다중 선택과는 별개입니다.
3. **Show Nodes**로 내부 그래프를 열어 노드·연결을 편집합니다. 노드를 선택하면 오른쪽 Inspector에서 로봇 설정도 변경할 수 있습니다. **Unbundle**은 원래 노드를 다시 펼칩니다. 노드 추가와 그래프 사이의 연결 편집은 Unbundle 후 진행합니다.
4. **Run Graph는 캔버스 전체의 실제 노드**를 실행합니다. 번들 카드는 표시용이며, 내부 노드 ID·핀·외부 연결·FANUC Guard 검사는 보존됩니다. 접힌 그래프의 외부 연결은 삭제할 수 없는 점선으로 표시합니다. 실행 중에는 구조와 설정 변경을 잠급니다.
5. **Save**는 번들 내부까지 포함한 `vlagraph-workspace.json`을 다운로드합니다. **Load → JSON import**로 복원합니다. 중복 ID·알 수 없는 노드 유형·없는 연결 대상·잘못된 번들 구성을 거부합니다. 자동 저장은 하지 않습니다.

별도로 실행 중인 수정 대상 프론트는 **http://localhost:9174**입니다. 비교 프로젝트의 9173과 구분해 갱신하려면 다음 명령을 사용합니다.

```powershell
docker compose -p vlagraph_current -f docker-compose.yml -f docker-compose.current.yml up -d --build --no-deps frontend
```

프론트 검증은 `frontend` 폴더에서 `node --test tests/*.test.mjs`와 `npm.cmd run build`를 실행합니다. `GRAPH_API_BASE=http://localhost:9000` 환경변수를 지정하면 로봇을 움직이지 않는 Prompt→Print 백엔드 연동 검증도 함께 실행합니다.

## 실행

한 씬의 여러 로봇을 공유 월드 상태에서 병렬 추론하려면 [다중 로봇 가이드](docs/multi-robot-world-inference-ko.md)와 [FANUC 2대·AMR 2대 예제 그래프](examples/graphs/multi_robot_world_inference_graph.json)를 사용하세요. 로봇별 작업·모델·센서를 연결하고 AMR 동시 제어 및 FANUC 후보의 준비·기존 Guard 경로를 제공합니다.

FANUC·AMR 라이브 관측과 차동구동 AMR의 시뮬레이션 주행은 [실행 가이드](docs/amr-live-world-model-ko.md)를 참고하세요. [라이브 주행 그래프](examples/graphs/amr_live_navigation_graph.json)는 관측·점유 격자 경로 계획·주행을 연결하며 기본값은 미리보기입니다. Isaac Sim 쪽에서 설정 파일과 `start_world_model_bridge.py` launcher를 사용해야 합니다.

기본 Compose 프로젝트 이름은 `docker-compose.yml`의 `name: vlagraph`로 고정합니다. 폴더를 이동하거나 복사해도 기존 `finetune_stub` 등 같은 프로젝트의 컨테이너를 이어서 관리합니다. `container name is already in use`가 나오면 별도로 지정한 `-p` 또는 `COMPOSE_PROJECT_NAME`을 확인하세요. 기존 전체 스택은 `vlagraph`를 사용합니다. 9174 전용 프론트는 위의 `-p vlagraph_current ... --no-deps frontend` 명령으로 갱신합니다.

```bash
docker compose up --build
```

Windows에서 RealSense D435i를 함께 사용할 때는 Docker 컨테이너가 USB 카메라를 직접 보지 못할 수 있으므로 host bridge를 먼저 올리는 스크립트를 사용합니다.

```powershell
.\scripts\docker-compose-up.ps1
```

백그라운드로 올리려면:

```powershell
.\scripts\docker-compose-up.ps1 -Detached
```

프론트: `http://localhost:9173`
백엔드: `http://localhost:9000`
VLA Stub: `http://localhost:9061`
GR00T Stub: `http://localhost:9071`
FineTune Stub: `http://localhost:9040`
STT: `http://localhost:9090`

## 메인 랜딩 페이지

별도 랜딩 페이지와 회원 로그인을 실행하려면 [landing/README.md](landing/README.md)의 안내에 따라 `landing/.env`를 설정하고 아래 명령을 실행하세요. 기존 그래프 앱은 계속 9173 포트를 사용합니다.

```powershell
docker compose --project-directory landing -f landing/docker-compose.yml up --build
```

랜딩 페이지: `http://localhost:5174` — 로그인 API는 `http://localhost:8001`, 로그인 후 **시연 화면 열기**를 누르면 그래프 앱 `http://localhost:9173`으로 이동합니다.

GPU 스켈레톤 서버까지 켜려면:
```bash
docker compose --profile gpu up --build
```

## 실제 OpenVLA로 바꿀 때
현재 backend는 `ROUTE_VLA=http://vla_stub:9060` 으로 연결됩니다.
실서버로 바꾸려면 `docker-compose.yml`에서 아래만 바꾸면 됩니다.

```yml
environment:
ROUTE_VLA: http://openvla_real:8055
```

## NVIDIA Isaac GR00T 노드
- **NVIDIA GR00T Infer** 노드는 `Prompt` / `Image` / `Robot State` / `LiDAR` 입력을 받아 `Action`을 출력합니다.
- 프론트 상단 **Graph Template** 메뉴에서 `OPENVLA Graph` 또는 `GR00T Graph` 를 선택해 기본 파이프라인을 바로 불러올 수 있습니다.
- Isaac Sim의 `/World/KyleRobot`과 GR00T 그래프를 처음부터 실행하는 순서는 [Isaac Sim KyleRobot + GR00T 실행 가이드](docs/isaacsim-gr00t-kyle-runbook.md)에 정리되어 있습니다.
- 실제 NVIDIA Isaac-GR00T `PolicyServer`를 VLA Graph에 붙이는 방법은 [실제 NVIDIA Isaac GR00T 서버 연결](docs/gr00t-real-policy-adapter.md)에 정리되어 있습니다.
- 사람 동작 영상에서 포즈/모션 trace를 뽑아 모방학습 또는 GR00T fine-tune으로 연결하는 방법은 [영상 기반 모션 인식/학습 파이프라인](docs/video-motion-learning-pipeline.md)에 정리되어 있습니다.
- 프론트 상단 **Graph Template** 메뉴에서 `Motion Learning Graph`를 선택하면 영상 업로드와 모션 trace 추출을 그래프 노드로 실행할 수 있습니다.
- 기본 GR00T 실행은 실제 `gr00t_policy_adapter`로 연결되며, stub fallback은 꺼져 있습니다.
- `.env`의 `COMPOSE_PROFILES=gr00t-real` 때문에 `docker compose up -d --build`만 실행해도 실제 GR00T 정책 서버와 HTTP adapter가 함께 올라갑니다.
  - `ROUTE_GR00T_STUB=http://gr00t_stub:9070`
  - `ROUTE_GR00T_REAL=http://gr00t_policy_adapter:9072`
  - `ENABLE_REAL_GR00T=true`
  - `GR00T_FALLBACK_TO_STUB=false`
  - `COMPOSE_PROFILES=gr00t-real`
  - 노드의 `endpoint`에 직접 `http://<host>:<port>`를 넣으면 환경변수보다 우선합니다.
- 실제 Isaac-GR00T `PolicyServer`는 ZMQ 기반이므로, VLA Graph의 HTTP `/infer`와 연결하려면 `services/gr00t_policy_adapter` 어댑터를 사용합니다. Compose profile은 `gr00t_policy_server`와 `gr00t_policy_adapter`를 같은 Isaac-GR00T 런타임 이미지로 실행합니다.
- 기본 모델명은 `nvidia/GR00T-N1.7-3B`이고, `embodimentTag`, `inferenceMode`, `actionHorizon`, `actionFormat`, `unityCommand`를 함께 전달합니다.
- NVIDIA 공식 배포 가이드는 GR00T N1.7을 PyTorch 또는 TensorRT로 로컬 배포하는 흐름을 설명합니다: https://github.com/NVIDIA/Isaac-GR00T/blob/main/scripts/deployment/README.md
- 하드웨어 기준은 NVIDIA 문서상 추론 최소 16GB+ VRAM, CUDA 12.6+이며, 실시간 제어 주파수는 플랫폼과 TensorRT 사용 여부에 따라 달라집니다: https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md

### MediaPipe -> Isaac Sim G1 조인트 변환

`C:\mediapipe_omniverse_humanoid_ext`에서 검증한 MediaPipe joint 변환 기능은 VLA Graph 백엔드 쪽으로 흡수했습니다. 전체 프로젝트를 `C:\isaac_exts`에 복사하지 않고, 변환 기능만 `backend/executors/mediapipe_g1_mapping.py`에 두는 구조입니다.

현재 흐름:

```text
MediaPipe / RealSense / RGB-D Pose Fusion
-> backend/executors/mediapipe_g1_mapping.py
-> G1 named joint targets / joint_target_map / joint_groups
-> Robot Execute(mode=omniverse)
-> C:\isaac_exts\robotis.omy.motion VLA Graph bridge
-> Unitree G1 articulation
```

`RGB-D Pose Fusion` 결과에는 이제 `g1_named_targets`, `joint_target_map`, `action`이 포함됩니다. `Motion Retarget`은 이 값이 있으면 우선 사용하고, 없으면 기존 retarget 로직으로 fallback합니다.

## Model Sync 노드
- 웹에서 받은 모델 폴더를 Docker volume `vlagraph_gr00t_models`의 `/models` 아래로 복사하는 준비용 노드입니다.
- backend 컨테이너는 `C:/Users/0/Downloads`를 `/host_downloads`로 읽고, `gr00t_models` volume을 `/models`로 씁니다.
- 프론트 상단 **Graph Template** 메뉴에서 `Model Sync Graph`를 선택하거나, 팔레트의 `MODEL > Model Sync` 노드를 추가해서 실행합니다.
- Cosmos 예시:
  - `sourcePath=/host_downloads/Cosmos-Reason2-2B`
  - `targetName=Cosmos-Reason2-2B`
  - `requiredFiles=config.json`
- GR00T 예시:
  - `sourcePath=/host_downloads/GR00T-N1.7-3B`
  - `targetName=GR00T-N1.7-3B`
  - `requiredFiles=config.json,processor_config.json,statistics.json`
- Windows 경로 `C:\Users\0\Downloads\Cosmos-Reason2-2B`를 입력해도 backend가 `/host_downloads/Cosmos-Reason2-2B`로 매핑합니다.

## 다음 확장 추천
1. edge handle 기반 자동 input resolution
2. 학습 로그 SSE 스트리밍
3. checkpoint selector 노드
4. dataset split / eval 노드
5. robot action viewer 패널

## Sensor 노드
- `Camera Sensor`: 브라우저 카메라를 켜고 프레임을 캡처해서 `image` / `sensor` 출력
- `Remote Image Sensor`: 원격 스냅샷 URL, MJPEG, RTSP 스트림에서 프레임 1장을 가져와 `image` / `sensor` 출력
- `Remote LiDAR Sensor`: 원격 JSON LiDAR 데이터를 가져와 `lidar` / `sensor` 출력
- `Fused Sensor`: 카메라 캡처 또는 원격 이미지와 원격 LiDAR를 함께 묶어 `image` / `lidar` / `sensor` 출력
- 기존 `Sensor Input` 노드는 호환성 유지용으로 남아 있고, 새 그래프에서는 개별 센서 노드 사용을 권장
- `remoteImageTransport`: `snapshot`, `mjpeg`, `rtsp`
- `rtspTransport`: `tcp` 또는 `udp`
- RTSP는 백엔드 컨테이너의 `ffmpeg`로 프레임 1장을 캡처
- `lidarFormat`: `auto`, `pointcloud`, `laser_scan`

## Unity WebSocket 연결
- `Robot Execute` 노드의 `mode`를 `unity`로 두고 `unityEndpoint=ws://<unity-host>:9081/robot` 로 설정
- 백엔드가 Docker 컨테이너에서 실행 중이면 macOS / Windows 기준으로 `unityEndpoint=ws://host.docker.internal:9081/robot` 를 쓰는 편이 가장 안전함
- `unityScene` / `unityObject` 는 Unity의 실제 Scene 이름과 제어할 GameObject 이름에 맞춰야 함
- Unity 기본 Scene 예시는 `VALConnect`, 기본 타겟 예시는 `RobotArm`
- 백엔드는 `request_id`를 포함한 JSON 명령을 Unity로 보내고 ACK 응답을 기다림
- Unity 샘플 수신기는 [UnityRobotWebSocketServer.cs](/Users/marlboro/Desktop/vlagraph/examples/unity/UnityRobotWebSocketServer.cs) 에 포함
- NavMeshAgent/휴머노이드 이동체를 같은 프로토콜로 연결하려면 [UnityHumanoidRobotBridge.cs](/Users/marlboro/Desktop/vlagraph/examples/unity/UnityHumanoidRobotBridge.cs) 예제를 사용할 수 있음
- 같은 브리지 스크립트에서 `enableDirectVlaClient=true` 로 두면 `ws://.../infer` 직접 접속형 VLA 루프도 함께 실험할 수 있음
- `command` 와 `action` payload는 로봇 종류에 따라 달라질 수 있고, 현재 예제는 `unityRobotType` / `unityControllerMode` 메타데이터와 함께 여러 Unity 명령 세트를 지원함
- `unityCommand` / `unityRobotType` / `unityControllerMode` 는 기본값이고, 액션 payload 안에 같은 메타데이터가 있으면 런타임에서 그 값으로 덮어쓸 수 있음
- 바로 테스트할 수 있는 샘플 그래프 JSON은 `examples/graphs/unity_articulated_arm_graph.json`, `examples/graphs/unity_mobile_base_graph.json`, `examples/graphs/unity_humanoid_graph.json` 에 포함
- GR00T 기반 휴머노이드 샘플은 `examples/graphs/unity_gr00t_humanoid_graph.json` 에 포함
- 이 샘플은 Unity Editor / Standalone 용이며 WebGL 서버 역할은 지원하지 않음

### Unity <-> Backend 실시간 상태 공유 (추가)
- 백엔드: `ws://<backend>/ws/unity/state?client_id=<id>` 로 Unity 상태를 수신해 메모리에 저장
- REST 조회: `GET /api/unity/state?client_id=<id>` 로 최신 상태 확인, `GET /api/unity/state/clients` 로 등록된 client 목록 확인
- `QUIET_UNITY_SENSOR_ACCESS_LOGS=true` 와 `QUIET_WEBSOCKET_LIFECYCLE_LOGS=true` 가 기본값이라 고주파 상태/이미지 WebSocket 로그는 조용하게 처리됨
- Unity 클라이언트: `examples/unity/UnityGraphApiClient.cs`에 `sendStateContinuously` 옵션 추가됨
  1. `sendStateContinuously=true`, `stateSendIntervalSec=0.1` 등 설정
  2. 제어 대상 오브젝트 이름을 `unityObject` 로 맞춘 뒤, `stateClientId` 로 식별자 지정
  3. 실행하면 포지션/회전/그리퍼 상태를 주기적으로 백엔드 WebSocket에 전송
- `api/unity/run` 에서 최신 Unity 상태를 그대로 쓰려면 `use_unity_state=true`, `state_client_id=<id>` 를 함께 보내고, 그 `state_client_id` 는 위 WebSocket 전송에 쓰는 값과 같아야 함
- 이 상태는 다른 그래프 노드나 외부 모니터링 도구에서 REST로 폴링해 로봇/씬 상태를 확인하거나 동기화에 활용할 수 있음
- 그래프에서 보기: 새 노드 **Robot State (Unity)** 를 추가하고 `clientId`(예: `robot1`)를 맞춘 뒤 `Print` 노드에 연결하면 실행 결과 탭에서 최신 Unity 로봇 상태를 바로 확인할 수 있음

### 프론트 `Run Graph` 연속 런타임
- 프론트의 `Run Graph` 는 이제 기본적으로 `/api/graph/runtime/start` 를 호출해 그래프 세션을 **서버 메모리**에 올림
- 그래프에 `Robot State (Unity)` 노드와 `Robot Execute(mode=unity)` 노드가 함께 있고 `clientId` / `unityStateClientId` 가 맞으면, 이 세션은 단발 실행이 아니라 **반복 런타임**으로 동작
- 동작 순서
  1. 백엔드가 최신 Unity 상태를 읽어 그래프를 1회 실행
  2. `Robot Execute` 가 Unity로 명령을 전송
  3. Unity가 상태 WebSocket(`/ws/unity/state`) 으로 처리 결과를 다시 보냄
  4. 백엔드가 새 상태를 받은 뒤 다음 회차 그래프를 다시 실행
- 프론트의 `Cancel Run` 은 `/api/graph/runtime/{session_id}/cancel` 을 호출하고, 백엔드는 현재 사이클 정리 후 그래프를 메모리에서 내림
- `Robot State (Unity)` 루프 조건이 없는 그래프는 세션 API를 쓰더라도 자동으로 **1회 실행 후 종료**됨
- 상태 조회 API
  - 시작: `POST /api/graph/runtime/start`
  - 조회: `GET /api/graph/runtime/{session_id}`
  - 취소: `POST /api/graph/runtime/{session_id}/cancel`

백엔드가 보내는 명령 예시:

```json
{
  "type": "robot_command",
  "request_id": "8c1e8d4a-3cce-4b67-8b4f-2b94d734a74d",
  "scene": "VALConnect",
  "object": "RobotArm",
  "command": "apply_action",
  "action": {
    "translation": [0.01, 0.0, 0.02],
    "rotation": [0.0, 0.0, 0.0],
    "gripper": "open",
    "terminate": false
  }
}
```

Unity ACK 예시:

```json
{
  "ok": true,
  "request_id": "8c1e8d4a-3cce-4b67-8b4f-2b94d734a74d",
  "applied": true,
  "message": "Unity command applied.",
  "scene": "VALConnect",
  "target": "RobotArm",
  "command": "apply_action"
}
```

로봇별 명령 예시:

```json
{
  "type": "robot_command",
  "request_id": "arm-001",
  "scene": "VALConnect",
  "object": "Arm01",
  "robot_type": "articulated_arm",
  "controller_mode": "joint",
  "command": "apply_joint_targets",
  "action": {
    "joint_targets": [0.0, 15.0, -10.0, 0.0, 30.0, 5.0],
    "gripper": "open"
  }
}
```

```json
{
  "type": "robot_command",
  "request_id": "base-001",
  "scene": "Warehouse",
  "object": "AGV01",
  "robot_type": "mobile_base",
  "controller_mode": "base",
  "command": "apply_base_velocity",
  "action": {
    "linear_x": 0.2,
    "linear_z": 0.8,
    "angular_yaw": 0.15,
    "stop": false
  }
}
```

```json
{
  "type": "robot_command",
  "request_id": "humanoid-001",
  "scene": "Campus",
  "object": "Humanoid01",
  "robot_type": "humanoid",
  "controller_mode": "locomotion",
  "command": "apply_humanoid_step",
  "action": {
    "move_x": 0.1,
    "move_z": 0.9,
    "turn_yaw": 0.1,
    "speed": 1.0,
    "stop": false
  }
}
```

사용 순서:
1. Unity 프로젝트에 `Assets/Scripts/UnityRobotWebSocketServer.cs` 로 복사
2. 빈 GameObject에 스크립트를 붙이고 WebSocket 서버를 `9081` / `/robot` 로 실행
3. 제어할 오브젝트 이름을 `RobotArm` 으로 두거나 `defaultTargetObject` / `unityObject` 를 같은 이름으로 맞춤
4. 그래프에서 `Safety Clamp -> Robot Execute(mode=unity, unityEndpoint=ws://host.docker.internal:9081/robot, unityScene=VALConnect, unityObject=RobotArm)` 로 연결
5. 백엔드 환경에 `websocket-client` 가 설치되어 있는지 확인 후 실행

휴머노이드/NavMeshAgent 예시:
1. Unity 프로젝트에 `Assets/Scripts/UnityHumanoidRobotBridge.cs` 로 복사
2. `NavMeshAgent` 가 붙은 캐릭터나 제어 대상에 스크립트를 붙이고 `controlledTransform` / `agent` / `defaultTargetObject` 를 맞춤
3. 관절 로봇이면 `articulatedJoints` 를 순서대로 연결하고 `unityRobotType=articulated_arm`, `unityControllerMode=joint` 를 권장
4. 이송 로봇이면 `unityRobotType=mobile_base`, `unityControllerMode=base` 를 권장
5. 휴머노이드면 `unityRobotType=humanoid`, `unityControllerMode=locomotion` 를 권장
6. 백엔드의 `Robot Execute(mode=unity)` 는 그대로 유지하고 `unityObject`, `unityCommand`, `unityRobotType`, `unityControllerMode` 를 Unity 쪽 설정과 맞춤
7. 반복 런타임에서 `Robot State (Unity)` 를 함께 쓸 경우 `sendStateContinuously=true`, `backendBaseUrl`, `stateClientId`, `unityStateClientId` 를 같은 값으로 맞춤
8. 직접 VLA 추론 루프를 같이 쓰려면 `enableDirectVlaClient=true`, `vlaWsUrl`, `goal`, `vlaInstruction` 을 설정하고 필요하면 `simCamera` / `includeCameraImage` 도 연결

## Omniverse WebSocket 연결
- `Robot Execute` 노드의 `mode`를 `omniverse`로 두고 `omniverseEndpoint=ws://<omniverse-host>:9900/omniverse` 로 설정
- 백엔드가 Docker 컨테이너에서 실행 중이면 macOS / Windows 기준으로 `omniverseEndpoint=ws://host.docker.internal:9900/omniverse` 를 먼저 시도하는 편이 안전함
- `omniverseUrl` 은 Nucleus/Stage 식별자, `omniverseStage` 와 `omniversePrimPath` 는 실제 적용 대상 경로를 기록하는 용도로 함께 전달됨
- 백엔드는 `request_id`를 포함한 JSON 명령을 Omniverse WebSocket 서버로 보내고 ACK 응답을 기다림
- 현재는 **Robot Execute(mode=omniverse)** 의 명령 실행까지 지원하며, Unity처럼 별도 **Robot State (Omniverse)** 노드는 아직 없음
- Isaac Sim 쪽 연결 예제는 `examples/isaacsim/VlaGraphIsaacSimBridge.py` 에 포함
- GR00T + Isaac Sim 샘플 그래프는 `examples/graphs/isaacsim_gr00t_humanoid_graph.json` 에 포함

Isaac Sim 브리지 빠른 실행:

```python
import os

os.environ["VLAGRAPH_BACKEND_BASE"] = "http://localhost:9000"
os.environ["VLAGRAPH_CLIENT_ID"] = "isaacsim01"
os.environ["VLAGRAPH_ROBOT_PRIM"] = "/World/KyleRobot"
os.environ["VLAGRAPH_ROBOT_TYPE"] = "humanoid"
os.environ["VLAGRAPH_CONTROLLER_MODE"] = "locomotion"

# KyleRobot 아래에 /World/KyleRobot/VlaGraphCamera 를 만들고
# 그 화면을 백엔드 이미지 저장소로 전송
os.environ["VLAGRAPH_SEND_IMAGE"] = "true"
os.environ["VLAGRAPH_AUTO_ATTACH_CAMERA"] = "true"
os.environ["VLAGRAPH_CAMERA_WIDTH"] = "640"
os.environ["VLAGRAPH_CAMERA_HEIGHT"] = "360"
os.environ["VLAGRAPH_IMAGE_INTERVAL_SEC"] = "0.25"
os.environ["VLAGRAPH_CAMERA_LOCAL_POSITION"] = "0,2.55796,-4.08236"
os.environ["VLAGRAPH_CAMERA_LOCAL_ROTATION_DEG"] = "-194.14137,1.0457,-179.11757"
os.environ["VLAGRAPH_CAMERA_LOCAL_SCALE"] = "1,1,1"
os.environ["VLAGRAPH_PRINT_PROMPTS"] = "true"

exec(open(r"C:/Users/0/Desktop/vlagraph/examples/isaacsim/VlaGraphIsaacSimBridge.py").read())
```

`Errno 10048` 이 보이면 9900 포트에 이전 브리지 인스턴스가 이미 떠 있는 상태입니다. Isaac Sim을 재시작하거나, Script Editor에서 다른 포트로 실행합니다:

```python
import os

os.environ["VLAGRAPH_OMNIVERSE_PORT"] = "9900"
exec(open(r"C:/Users/0/Desktop/vlagraph/examples/isaacsim/VlaGraphIsaacSimBridge.py").read())
```

이 경우 `Robot Execute.omniverseEndpoint` 도 `ws://host.docker.internal:9900/omniverse` 로 바꿉니다.

Isaac Sim Python 환경에 `websockets`가 없다면 먼저 설치:

```bash
./python.sh -m pip install websockets pillow
```

그래프 설정:

```text
Robot State (Unity): clientId=isaacsim01
NVIDIA GR00T Infer: unityCommand=apply_humanoid_step
Robot Execute: mode=omniverse
Robot Execute: omniverseEndpoint=ws://host.docker.internal:9900/omniverse
Robot Execute: omniverseStage=/World
Robot Execute: omniversePrimPath=/World/KyleRobot
Robot Execute: omniverseCommand=execute_policy
Remote Image Sensor: remoteImageUrl=http://localhost:9000/api/unity/image?client_id=isaacsim01
```

FastAPI 백엔드가 `Robot Execute(mode=omniverse)` 를 실행할 때 `Prompt -> GR00T/OpenVLA Infer` 에 들어간 문장은 Isaac Sim WebSocket payload의 `prompt` / `instruction` 으로 함께 전달됩니다. Isaac Sim 브리지는 이 값을 `last_prompt` 로 저장하고 다음 `Robot State (Unity)` 상태 payload에도 포함합니다.

백엔드가 보내는 명령 예시:

```json
{
  "type": "omniverse_command",
  "request_id": "3ef6166d-7f8d-46b7-a06e-9a7047f62cf5",
  "url": "omniverse://localhost/Projects/Robot",
  "stage": "/World",
  "prim_path": "/World/KyleRobot",
  "command": "execute_policy",
  "action": {
    "translation": [0.01, 0.0, 0.02],
    "rotation": [0.0, 0.0, 0.0],
    "gripper": "open",
    "terminate": false,
    "type": "delta_action"
  }
}
```

Omniverse ACK 예시:

```json
{
  "ok": true,
  "type": "omniverse_ack",
  "request_id": "3ef6166d-7f8d-46b7-a06e-9a7047f62cf5",
  "applied": true,
  "message": "Omniverse command applied.",
  "stage": "/World",
  "prim_path": "/World/KyleRobot",
  "command": "execute_policy"
}
```

사용 순서:
1. Omniverse 쪽에서 WebSocket 서버를 열고 `/omniverse` 같은 엔드포인트에서 위 JSON 명령을 수신하도록 준비
2. 그래프에서 `Robot Execute(mode=omniverse)` 로 전환
3. `omniverseEndpoint`, `omniverseUrl`, `omniverseStage`, `omniversePrimPath`, `omniverseCommand` 를 환경에 맞게 설정
4. `Run Graph` 실행 후 `Robot Execute` 노드의 latest run/message로 ACK 결과 확인

## Unity에서 백엔드 호출
- Unity는 `http://localhost:9000/health`, `http://localhost:9000/api/graph/run`, `http://localhost:9000/api/unity/run` 을 직접 호출할 수 있음
- 프론트의 `Run Graph` 는 위 `api/graph/run` 이 아니라 **세션형 런타임 API**(`api/graph/runtime/start`)를 사용함
- `api/unity/run` 은 Unity 클라이언트에서 쓰기 쉽게 고정 파이프라인 `prompt -> openvla_infer -> action_decode -> safety_clamp -> robot_execute(mode=unity)` 를 내부에서 조립해 실행함
- `use_unity_state=true` 이면 내부 상태 노드로 `Robot State (Unity)` 를 사용하고, 아니면 요청 body의 `robot_state` 값을 사용함
- `api/unity/run` 은 현재도 **1회 실행용 편의 API** 이며, 프론트의 반복 런타임과는 별도임
- Unity Editor / Standalone 에서 같은 머신의 Docker 백엔드를 부를 때 `backendBaseUrl=http://localhost:9000` 으로 두면 됨
- 예제 클라이언트는 [UnityGraphApiClient.cs](/Users/marlboro/Desktop/vlagraph/examples/unity/UnityGraphApiClient.cs) 에 포함

간단한 요청 예시:

```json
{
  "prompt": "walk to CheckpointA",
  "image": "http://localhost:9000/uploads/<uploaded-image>.png",
  "use_unity_state": true,
  "state_client_id": "robot1",
  "robot_state": {
    "gripper_open": true,
    "eef_pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
  },
  "model_name": "openvla-stub",
  "unity_endpoint": "ws://host.docker.internal:9081/robot",
  "unity_scene": "VALConnect",
  "unity_object": "RobotArm"
}
```

curl 테스트:

```bash
curl -X POST http://localhost:9000/api/unity/run \
  -H 'Content-Type: application/json' \
  -d '{
    "prompt": "walk to CheckpointA",
    "use_unity_state": true,
    "state_client_id": "robot1",
    "unity_endpoint": "ws://host.docker.internal:9081/robot",
    "unity_scene": "VALConnect",
    "unity_object": "RobotArm"
  }'
```
