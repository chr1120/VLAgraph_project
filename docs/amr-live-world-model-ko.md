# FANUC·AMR 라이브 월드모델과 시뮬레이션 주행

한 씬에 여러 종류·여러 대의 로봇을 함께 두는 구성은 [다중 로봇 월드 상태·병렬 추론 가이드](multi-robot-world-inference-ko.md)를 참고한다. AMR 제어 상태는 이제 로봇별로 분리하며, `navigation_maps` 출력으로 각 로봇의 지도를 제공한다.

`Robot Observations Read → AMR Path Plan → AMR Navigate`로 라이브 관측과 차동구동 AMR의 경로 추종을 연결한다. FANUC M-710iC/20L의 관절 관측은 같은 `WorldState.robots`에 포함한다. 이 경로는 Isaac Sim용이며 FANUC 실행은 기존 FANUC 전용 노드·가드를 사용한다.

## Isaac Sim에서 시작

1. [설정 예제](../examples/isaacsim/mixed_robot_world_model.example.json)를 별도 파일로 복사하고 현재 Stage에 맞게 수정한다. 로봇 prim·articulation 경로, 실제 DOF 이름, 바퀴 반경·좌우 간격·회전 부호, 차체 외접 반경을 지정한다. FANUC의 `joint_bindings`는 명령 순서가 아니라 실제 DOF 이름에 연결한다. 레일이 있으면 `base_motion: rail`과 prismatic 관절 매핑을 추가한다.
2. `navigation_map.stage_identifier`는 Isaac Sim의 `omni.usd.get_context().get_stage().GetRootLayer().identifier`와 정확히 맞춘다. 점유 격자의 원점·해상도·셀은 **Z-up 월드 미터** 단위로 작성한다. `cells[y][x]`는 `0`이 확인한 주행 가능 공간, `1`은 점유, `-1`은 미관측이다. 예제의 전체 `0` 격자는 가상의 평지 예시이며 실제 장면의 통행 가능 공간을 뜻하지 않는다. 지도 밖·미관측 셀은 경로에 사용할 수 없다.
3. `ground_prim_paths`에는 주행면만 지정한다. 이 목록과 선택 AMR의 하위 prim을 제외한 활성 USD collider의 월드 AABB를 매 관측·제어 단계에서 지도에 추가한다. 다른 로봇의 collider도 장애물로 포함한다. AABB 투영은 보수적인 2차원 근사이며 계단·경사·공중 통로 등의 통행 가능성을 판정하지 않는다.
4. Stage를 열고 Play로 물리 시뮬레이션을 시작한다. AMR의 좌·우 바퀴에는 속도 제어용 USD drive를 설정한다. 베이스 로컬 축은 전방 `+X`, 상방 `+Z`이며 Y-up Stage에서는 이에 맞는 월드 자세가 필요하다. 첫 구현은 차동구동 평면 AMR을 지원하고 ackermann·omnidirectional AMR은 주행 요청을 거부한다.
5. Isaac Sim Script Editor에서 다음을 실행한다. 경로는 현재 작업 폴더 기준이며 설정 파일 경로는 수정한 파일로 바꾼다.

```python
import os
import runpy

os.environ["VLAGRAPH_WORLD_MODEL_CONFIG"] = (
    r"C:\Users\0\Desktop\vla_chr\VLAgraph_project\examples\isaacsim\mixed_robot_world_model.example.json"
)
runpy.run_path(
    r"C:\Users\0\Desktop\vla_chr\VLAgraph_project\examples\isaacsim\start_world_model_bridge.py",
    run_name="__main__",
)
```

설정은 시작 시 읽는다. 변경 후 같은 launcher를 다시 실행하면 이전 브리지의 활성 AMR 명령을 정지하고 브리지를 교체한다. launcher는 `backend.world_model.amr_navigation`을 읽을 수 있도록 저장소 경로를 설정한다. 일반 `VlaGraphIsaacSimBridge.py`만 다시 실행하면 이 어댑터는 붙지 않는다. 기존 FANUC 전용 9910 브리지와는 별도의 9900 브리지다.

## 그래프 사용

[라이브 주행 그래프](../examples/graphs/amr_live_navigation_graph.json)를 Load의 JSON import로 불러온다. 먼저 `robotPrimPath`·브리지 주소와 목표 `goalM: [x, y]`를 지정한다. 기본 `enableExecution: false`에서는 관측·경로 계산만 수행한다. 출력 Plan을 확인하고 시뮬레이션 주행이 필요하면 `AMR Navigate`의 `enableExecution`을 켜고 다시 실행해 새 관측으로 계획한다.

| 노드 | 입력·설정 | 결과 |
| --- | --- | --- |
| Robot Observations Read | 브리지 주소·선택 로봇 prim·sceneVersion | 전체 로봇 관측, 선택 로봇, 정규화한 WorldState, 점유 지도 |
| AMR Path Plan | WorldState·Navigation Map·Goal XY·marginM | 차체 반경과 여유를 반영한 A* 경로, 지도·에피소드·시작 위치·digest |
| AMR Navigate (Simulation) | Plan·enableExecution·navigationTimeoutSec | 비동기 주행 시작과 command_id. 접수 응답은 도착 확인이 아님 |
| AMR Status | command_id 연결 또는 commandId 입력 | running/completed/blocked/cancelled/failed/stopping과 완료 여부 |
| AMR Stop | command_id 연결 또는 commandId 입력 | 해당 명령의 바퀴 목표 속도를 0으로 설정하고 cancelled 처리 |

`AMR Navigate`는 바로 반환하고 Isaac Sim update 루프가 경로를 추종한다. 반환된 `command_id`를 보관해 `AMR Status`에서 진행 상태를 확인한다. 같은 command ID와 같은 계획을 다시 보내도 재실행하지 않는다. 다른 계획에 같은 ID를 쓰면 거부한다. 전송 후 응답이 유실되면 다른 서버로 주행 명령을 재전송하지 않고 ID로 조회하도록 오류를 반환한다.

그래프 런타임 세션은 AMR 명령을 전송 전에 등록하고 브리지 주소와 command ID를 보관한다. 노드가 반환한 뒤에도 상태를 조회하며 도착이 검증될 때까지 세션 완료를 기다린다. Cancel Run과 그래프 오류 종료는 해당 세션의 AMR에 정지를 요청한다. 전송 중 취소로 첫 정지가 명령을 찾지 못하면 노드 응답 이후 다시 요청한다. 정지를 확인하지 못하면 세션은 명령 ID와 함께 오류를 표시한다. 세션 조회 응답의 `amr_commands`에서 각 명령의 상태·완료 여부를 확인할 수 있다. 직접 `run_graph()`를 호출한 경우에는 세션 관리가 없으므로 별도 상태 조회·정지 노드를 사용한다.

최종 목표에서 위치 오차 3cm 미만, 측정 선속도 0.02m/s 미만, 측정 각속도 0.03rad/s 미만이 **서로 다른 물리 스텝 3회** 유지되어야 `completed`·`completion_verified: true`가 된다. 단순 idle·ACK·중복 UI 프레임으로 완료를 만들지 않는다. FANUC·AMR의 모든 로봇 관측은 여전히 물리 예측 준비 여부를 뜻하지 않으며 `prediction_ready`는 `false`다.

주행 중 직접 추종할 구간에 장애물·미관측 셀이 들어오면 `blocked`로 정지한다. 새 관측과 계획으로 다시 실행한다. 관측 실패·기울어진 베이스·15초간 진행 없음·전체 시간 초과·물리 시간 정체·큰 스텝 간격·시뮬레이션 리셋·다른 제어 명령의 개입도 정지 또는 취소 사유다. 정지 명령 전송 자체가 실패하면 `stopping`을 유지하며 0 속도 명령을 재시도한다. 이는 물리적 제동 완료 인증이나 장애물의 미래 움직임 예측을 제공하지 않는다.

Stage 교체·물리 시간 리셋을 감지하면 이전 articulation 핸들을 먼저 폐기하고 새 에피소드로 전환한다. 새 물리 핸들이 준비되지 않으면 기존 명령은 `stopping`을 유지하며 새 에피소드의 핸들로 0 속도 명령만 재시도한다. 이전 주행을 다시 시작하지 않는다.

## 관측과 좌표

관절은 지정한 `joint_bindings`의 실제 DOF 이름으로 읽는다. 회전 관절은 rad·rad/s, 직선 관절은 Stage 길이 단위를 m·m/s로 변환한다. 라이브 베이스 자세·속도는 SingleArticulation에서 읽고, 속도 수집이 실패하면 값을 생략하고 `observation_error`를 남긴다. AMR은 유효한 물리 관측을 얻은 경우에만 `navigation_frame: stage`, `localization_valid: true`를 제공한다. map/odom 변환을 추정하지 않는다.

AMR 주행은 선택한 두 바퀴 DOF의 속도 목표만 갱신한다. root pose·관절 위치를 직접 이동시키지 않는다. API 선택은 [NVIDIA SingleArticulation 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html)의 관측·ArticulationAction 경로를 기준으로 한다. 설치된 Isaac Sim 버전의 API와 모델 drive 설정은 실제 장면에서 확인해야 한다.

저장 JSON을 `WorldState`로 직접 읽을 때도 단위 quaternion, 유일한 관절명·엔티티 ID·prim 경로, 선택 `robot`과 `robots` 목록의 일치를 검증한다. 확장 전 v1 JSON에 `robots` 필드가 없으면 선택 로봇 하나로 복원하며, 명시한 빈 목록이나 상충하는 관측은 거부한다.

## 검증 범위

자동 테스트는 A* 장애물 우회, 차체가 통과할 수 없는 좁은 통로·미관측 공간 거부, 좌표·단위·DOF 매핑, collider 지도 반영, 그래프 연결, 접수와 완료 구분, 제어 루프의 합성 기구학 도착, 중복 명령, 리셋·관측 실패·정지 재시도·응답 유실을 검사한다. PhysX 바퀴 접촉·미끄러짐·모터 drive의 실제 추종은 이 테스트로 확인하지 않는다. 현재 작업 환경에서는 실행 중인 Isaac Sim을 찾지 못해 실제 USD 장면의 라이브 주행 검증은 수행하지 않았다.
