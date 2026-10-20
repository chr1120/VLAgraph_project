# FANUC M-710iC/20L Isaac Sim 실행 템플릿

Graph Template에서 `FANUC M-710iC/20L Execute`를 선택합니다.

현재 Execute 템플릿에는 **Collection 공간 기억 대조 → 현재 USD 기반 충돌 검사 → 예측 가용성 확인 → Guard** 연결이 포함됩니다. 템플릿을 다시 선택해야 새 노드와 연결이 로드됩니다. 기존 저장 그래프는 자동 변경하지 않습니다.

1. FANUC 장면에서 Collection을 실행합니다. Collection의 Scene Scan에 FANUC 로봇 prim과 브리지 주소를 설정하고 저장 성공을 확인합니다. 기본 휴머노이드 prim으로 수집한 기록은 FANUC 기록으로 사용할 수 없습니다.
2. Execute 템플릿을 선택하고 `FANUC Collection Context`의 `memoryPath`를 Collection의 저장 경로와 같게 지정합니다. 기본값은 `spatial_memory/spatial_memory.jsonl`입니다.
3. Execute·새 Physics Snapshot·Scene Scan·USD Geometry 노드의 브리지 주소와 로봇 prim을 같은 대상에 맞춥니다. 기본 원격 주소는 기존 설정을 유지합니다.
4. LS 파일을 지정하고 실행하면 저장 기록과 새 관측을 대조합니다. `Collection 대조 결과`, `M-710 기하 충돌 결과`, `월드모델 예측 상태`, `M-710 실행 판단`을 확인합니다.

로봇·Stage·단위·축이 다르거나, 기록이 오래됐거나, 물체가 추가·삭제·이동했거나, 관측 물체의 실제 정적 충돌 형상이 없으면 충돌 scene을 내보내지 않습니다. 기존 기억의 접근 좌표·시각 바운딩박스를 충돌 형상으로 자동 변환하지 않습니다. 오래된 기록에 Stage·단위·축 정보가 없으면 최신 어댑터를 활성화한 뒤 다시 수집해야 합니다.

**이 연결은 물리 미래 예측 엔진의 구현 완료를 뜻하지 않습니다.** 현재 Prediction Handoff는 관절 궤적의 명목 기하 sweep과 입력 상태·장면의 일치를 확인하고, 별도 물리 워커가 없다는 결과를 Guard에 전달합니다. 연결된 월드모델 경로는 `static_simulation`을 선택해도 물리 예측 미검증으로 차단합니다. 완전한 자유 실행은 아직 허용하지 않습니다.

**마감 상태:** 검사·가드·일괄 진단의 개발 검증을 마쳤습니다. 실제 실행은 보정 실측 자료, 실행 중인 브리지의 어댑터 연결, L/CNT·공정 의미 및 물리 충돌·완료 검증이 남아 있어 보류입니다. [최종 마감 결과](validation/fanuc-m710-closeout-2026-10-20/summary.json)에 검사 수집 완료와 실행 검증 미완료를 각각 기록했습니다.

관련 검사를 한 번에 모으려면 `python scripts/validate_fanuc_bundle.py S05PROC.LS --candidate R01/tooldata.ls --usd Test8.usd --endpoint ws://127.0.0.1:9910/omniverse --output-dir validation/fanuc-run-001`을 사용합니다. 매번 새 출력 폴더를 지정합니다. 보정 후보·USD 형상·원본 LS·기존 Execute의 차단 동작·읽기 전용 브리지 검사를 수집하고 `summary.json`에 남은 조건을 모읍니다. 원본 Execute 템플릿은 저장·변경하지 않으며 실제 이동 명령을 보내지 않습니다. [사용법과 실제 일괄 결과](vlagraph-autonomous-world-model-design-ko.md#fanuc-일괄-검증)를 확인하십시오.

1. `FANUC M-710 LS 파일 업로드`에서 `.LS` 파일을 선택합니다.
2. `FANUC LS 파싱`이 `/MN`, `/POS` 레코드를 읽습니다.
3. `M-710 J1~J6 명령 준비`가 joint 레코드와 Cartesian 레코드를 전용 bridge action으로 변환합니다.
4. `FANUC Trajectory Preflight`가 시간·관절 제한·단위·누락 프레임·보정·L/CNT 재현 조건을 검사합니다. `M-710 상세 검사 결과`에서 보고서를 확인합니다.
5. `Execution Guard`가 검사한 궤적과 입력 궤적의 일치 여부 및 실행 범위를 확인하고 `M-710 실행 판단`에 허용·차단 사유를 출력합니다.
6. `FANUC M-710 Isaac Sim 실행`은 가드가 허용한 궤적만 설정된 브리지로 보냅니다. JSON 템플릿과 UI의 기존 원격 endpoint 설정은 유지하므로 실행할 PC에 맞는지 확인합니다.

```text
LS 업로드 → 파싱 → 궤적 준비 → 상세 검사 → 실행 가드 → Isaac Sim 실행 → 결과
                                  ↓           ↓
                              검사 보고서    실행 판단
```

실행 가드의 Inspector에서 `validationPolicy`를 선택합니다.

| 설정 | 처리 |
| --- | --- |
| `full` (기본값) | 전체 검증 요구. 현재 충돌·물리 예측이 미구현이므로 실행을 차단하고 이유를 표시합니다. |
| `static_simulation` | 정적 검사 통과 및 준비 노드의 실행 활성화 조건을 만족하면 Isaac Sim으로 전송합니다. 충돌·물리 예측 검증은 포함하지 않습니다. `omniverse` 모드에서만 사용할 수 있습니다. |

검사 노드 자체는 실행을 허가하지 않습니다. 가드가 보고서와 원본 action의 SHA-256을 비교하고 정적 검사를 다시 수행합니다. 실행 노드도 가드 출력의 일치를 확인하므로 직접 준비 노드에 연결하거나 가드 이후 궤적이 바뀌면 전송을 차단합니다. 차단·미입력 상태는 명시적인 `skipped` 결과로 출력합니다.

기존에 화면에서 편집·저장한 그래프는 자동으로 재배선하지 않습니다. `FANUC M-710iC/20L Execute` 템플릿을 다시 선택하거나 검사·가드를 직접 연결해야 새 흐름이 적용됩니다. `FANUC M-710 LS Preflight (Dev)`는 별도 실행 없는 개발·검증용으로 유지합니다.

Isaac Sim에서 `fanuc.m710ic20l.motion` 확장을 먼저 활성화합니다. 창의 bridge 상태가
`running`이고 로봇 상태가 `physics_ready=true`인지 확인하십시오. 초기화가 필요하면
bridge에 `{"command":"init"}`를 보내거나 확장 UI에서 로드 및 물리 초기화를 실행합니다.

`J1`~`J6` 레코드는 그대로 실행합니다. `XYZ/WPR` 레코드는 확장에 포함된 M-710 USD의
링크·축·관절 한계를 이용해 수치 IK로 변환합니다. 실제 UFRAME/UTOOL Stage 보정값이
없으므로 각 UF/UT 그룹의 첫 위치를 기준으로 이동과 회전을 상대 변환합니다. 기본 위치
스케일과 상대 회전 스케일은 각각 1.0이며, IK 기본 허용 오차는 위치 0.001m, 자세 1도입니다.
자세 가중치는 0.75로 설정되어 TCP 방향을 이전보다 강하게 추종합니다. `CONFIG : 'N U T, ...'`
값은 raw flag와 turn 값으로 보존되어 이후 configuration IK에 사용할 수 있습니다.
`uframeTransforms`에 로봇 IK 베이스 기준으로 정렬한 UFRAME 변환을 넣으면 상대 anchor 대신 절대 pose를
사용합니다. 값은 예를 들어 `{"4":{"xyzMm":[0,0,0],"wprDeg":[0,0,0]}}` 형태입니다.
`utoolTransforms`에는 실제 TCP offset/orientation을 같은 형식으로 넣을 수 있습니다.

컨트롤러 TOOLDATA의 UF/UT를 적용하기 전 `scripts/review_fanuc_calibration.py`로 모델명·프레임 번호·입력 오류를 확인할 수 있습니다. [실제 후보 비교](validation/fanuc-m710-calibration-candidates-2026-10-20.json)에서 R01은 M-710iC/20L 및 S05PROC의 UF1/UT1과 일치했고 R07은 M-20iA용이라 부적합입니다. R01도 컨트롤러 베이스→IK 베이스, TCP 규약, 외부축 영점 및 기준 자세 잔차를 확인해야 합니다. 도구는 후보를 자동 적용하거나 보정 완료로 인증하지 않습니다. [설계 문서의 후보 검토](vlagraph-autonomous-world-model-design-ko.md#ufut-초기-설정-후보-검토)에 사용법과 남은 확인 항목을 기록했습니다.

USD의 joint-frame 회전을 반영해 J3~J6의 회전축 부호를 계산합니다. 수치 IK가 이전 seed에서
140도 이상 떨어진 branch로 이동하면 configuration 유지 실패로 간주해 해당 Cartesian 프레임을
기록합니다. 통합 Execute 템플릿은 `requireAllIk=true`이므로 IK 실패나 미해결 위치가 있으면 전체 궤적을 실행 준비 상태로 표시하지 않습니다.
기본 `timingMode=source_speed`에서는 LS의 `mm/sec`와 `%` 속도로 timestamp를 생성하고,
`FINE/CNTn`은 `cnt_percent` 메타데이터로 bridge에 전달합니다. 기존처럼 전체 시간을 균등
분배하려면 `timingMode=fixed_total`과 `totalDurationSec`를 사용합니다.

이 IK는 UF/UT 그룹별 상대 이동 재현용입니다. FANUC 절대 좌표를 Isaac Stage에서 정확히
재현하려면 각 UFRAME의 Stage 변환과 UTOOL/TCP 오프셋을 별도로 입력해야 합니다.

독립 JSON 템플릿은 `examples/graphs/isaacsim_fanuc_m710ic20l_graph.json`에 있습니다.

## 월드모델 관측 기반 검증

Graph Template의 `FANUC M-710 World Observe`는 관절·외부축 상태와 작업 셀을 읽는 별도 템플릿입니다. `FANUC State Read`는 `get_state`만 요청하며, 장면 스캔은 독립 분기로 실행합니다. 관측 결과는 각 Print 노드에서 확인합니다.

기본 endpoint는 Docker 백엔드용 `ws://host.docker.internal:9910/omniverse`입니다. 호스트에서 직접 검증할 때는 다음 명령을 사용합니다.

```powershell
python scripts/validate_fanuc_observation.py --endpoint ws://127.0.0.1:9910/omniverse
```

전용 브리지의 `joint_positions_rad` 배열에서 J1~J6는 radian, `externalAxisJoint`는 meter입니다. 새 관측 노드는 명령 관절 순서와 실제 DOF 매핑을 검증하고 축별 단위를 명시합니다. `motion_active=false`를 작업 완료로 판정하지 않으며, 물리 시각·TCP·보정·명령 완료 정보가 없는 현재 관측은 미래 예측용 상태로 표시하지 않습니다.

- [관측 그래프 JSON](../examples/graphs/fanuc_m710_world_observe_graph.json)
- [실제 브리지 관측 검증 기록](validation/fanuc-m710-observation-2026-10-20.json)
- [월드모델 개발 설계의 FANUC 적용](vlagraph-autonomous-world-model-design-ko.md#fanuc-m-710-작업-셀-적용)

## 물리 스냅샷과 명령 완료 조회

전용 브리지 어댑터를 설치한 뒤, 현재 실행 중인 Isaac Sim의 Script Editor에서 다음 코드를 실행합니다. 기존 브리지에 관측·완료 추적 기능을 연결하며 물리를 재시작하지 않습니다.

```python
exec(open(r"C:\Users\0\Desktop\vla_chr\VLAgraph_project\examples\isaacsim\AttachFanucWorldModel.py", encoding="utf-8").read())
```

`FANUC M-710 Physics Snapshot` 템플릿은 관절 위치·목표·속도와 시뮬레이션 시각을 읽습니다. 툴 pose는 아직 USD 기반 플랜지 관측이며 보정된 물리 TCP로 취급하지 않습니다.

```powershell
python scripts/validate_fanuc_observation.py --endpoint ws://127.0.0.1:9910/omniverse --snapshot
```

기존 실행 노드의 `data.commandId`에 명령 ID를 지정하면 완료 추적을 활성화할 수 있습니다. `Execution Observe` 노드에 같은 ID를 설정해 한 번씩 상태를 조회합니다. 같은 논리 요청 재시도에는 같은 ID를 사용하며, 새 실행에는 새 ID가 필요합니다. 완료 판정은 보간 종료와 관절 오차·속도의 3개 물리 스텝 안정화를 요구합니다. 이 판정은 관절 명령 완료이며 작업 성공 판정은 후속 단계입니다.

설치·백업 경로와 완료 임계값은 [월드모델 설계](vlagraph-autonomous-world-model-design-ko.md#물리-스냅샷과-명령-완료-어댑터)에 기록했습니다. 현재 파일 설치와 자동 테스트는 완료했으며 실환경 스냅샷·완료 검증은 Script Editor 연결 후 진행해야 합니다.

## 실행 전 오프라인 궤적 검사

Graph Template의 `FANUC M-710 LS Preflight`를 선택하면 LS 업로드부터 궤적 준비·검사까지 실행할 수 있습니다. 실행 노드와 브리지 접속은 포함하지 않습니다. 이 템플릿은 `requireAllIk=true`로 미해결 위치 및 IK 실패 프레임을 검사하며, 관절 제한·시간·단위·UF/UT 보정·L/CNT 재현 조건을 보고합니다.

```powershell
python scripts/validate_fanuc_trajectory.py C:/path/to/program.LS --output preflight-report.json
```

초기 설정이 있으면 `--initial-settings C:/path/to/initial.LS`를 추가합니다. `--settings`의 `prepare`와 `validate` JSON 항목으로 보정값 및 속도·가속도 제한을 지정할 수 있습니다. 제한은 회전축 rad/s·rad/s², 외부축 m/s·m/s² 단위를 사용합니다. 미설정 제한과 충돌·물리 예측은 미검증으로 남습니다.

검사 오류가 있으면 실행용 action을 출력하지 않습니다. 정적 검사가 통과해도 출력은 preview 전용이며 실행 허가로 사용하지 않습니다. 기존 실행 템플릿을 사용할 때도 이제 `requireAllIk=true`라면 미해결 위치 레코드가 있는 궤적의 실행 준비 상태를 거부합니다.

- [오프라인 검사 그래프](../examples/graphs/fanuc_m710_preflight_graph.json)
- [S05PROC.LS 검사 결과](validation/fanuc-m710-preflight-2026-10-20.json)

## 기하 충돌 검사 분기

Execute 템플릿의 `FANUC Geometry Collision`은 준비된 궤적에 대해 명시적으로 설정한 링크 캡슐과 정적 장애물 박스를 검사합니다. 전체 작업 셀 모델이 아직 검증되지 않아 기본 `enabled=false`이며, `M-710 기하 충돌 결과`에 `not_requested`를 표시합니다. 실제 USD 형상 추출과 관절 좌표계 대조 및 팔 캡슐 자동 생성은 구현했습니다.

사용할 때는 노드의 `scene`에 베이스 pose, 링크 국소 캡슐 끝점·반경, 장애물 박스, 자기 충돌 제외 쌍을 입력하고 활성화합니다. 단위는 meter, 좌표계는 Z-up입니다. 외부축이 있으면 세계 방향과 기준 위치도 명시해야 합니다. [예제 입력](../examples/graphs/fanuc_m710_collision_example.json)은 임의 반경의 합성 모델이며 실제 로봇의 검증된 충돌 형상이 아닙니다.

표본에서 충돌·여유 거리 미달이 나오거나 표본 사이의 최대 이동 거리로 무충돌을 보증하지 못하면 가드가 전송을 차단합니다. `clear`는 입력한 기하 모델과 검사 구간에 한정한 결과이며 `full`의 물리 검증을 대체하지 않습니다. `enabled=true`인데 형상이 누락된 경우도 차단합니다. 기존 저장 그래프에는 템플릿을 다시 선택하거나 해당 분기를 연결해야 반영됩니다.

```powershell
python scripts/validate_fanuc_collision.py examples/graphs/fanuc_m710_collision_example.json --output proxy-report.json
```

snapshot을 입력하면 시작 관절값에서 첫 목표까지의 이동도 검사하며, 미입력 시에는 해당 이동을 미검증으로 남깁니다. 실제 mesh 포괄성, 동적 장애물과 물리 rollout은 아직 검사하지 않습니다. [합성 예제 검증 기록](validation/fanuc-m710-collision-proxy-2026-10-20.json)과 입력 계약은 [월드모델 설계](vlagraph-autonomous-world-model-design-ko.md#명시적-기하-모델의-충돌-진단)에 기록했습니다.

## USD 형상과 관절 연결 확인

공간 팔레트의 `FANUC USD Geometry`를 추가합니다. `source=file`에서는 백엔드가 접근 가능한 `usdPath`를, `source=bridge`에서는 FANUC 브리지 주소와 로봇 prim 경로를 설정합니다. 파일 입력은 pxr가 필요하고, 브리지 입력은 갱신된 어댑터 연결이 필요합니다. 현재 실행 중인 브리지는 새 명령을 거부하므로 Script Editor 연결 후 조회를 다시 확인해야 합니다.

```powershell
python scripts/export_fanuc_usd_geometry.py C:/path/to/Test8.usd --output collision-inventory.json
```

실제 개발 장면에서 인스턴스 내부를 포함해 9개 충돌체와 J1~J6 연결을 추출했습니다. 환경 장애물 후보는 0개이며 환경이 안전하다는 뜻은 아닙니다. [추출 기록](validation/fanuc-m710-usd-collision-inventory-2026-10-20.json)은 물리 동기화나 실제 충돌 검증을 통과한 결과가 아닙니다.

출력 `inventory`를 기존 Execute의 충돌 노드에 연결하면 검사를 활성화했을 때 관절 형상 누락과 출처 불일치를 거부합니다. 함께 입력하는 `scene.inventory_digest`는 inventory의 digest와 같아야 합니다. 상세 범위는 [USD 형상 추출 설계](vlagraph-autonomous-world-model-design-ko.md#실제-usd-충돌-형상-추출)를 참고합니다.

## 관절 좌표계 대조와 팔 캡슐 자동 생성

같은 그래프에 `FANUC USD Proxy`를 추가하고 Geometry의 `inventory`를 입력합니다. Proxy의 `scene` 출력을 기존 `FANUC Geometry Collision.scene`에 연결하고, Geometry의 `inventory`도 충돌 노드의 같은 이름 입력에 연결합니다. 기존 Prepare·Preflight·Guard·Execute 흐름을 그대로 사용합니다.

Proxy는 J1~J6의 부모 연결·부호 있는 축·원점·자식 frame·관절 제한과 베이스/외부축 연결을 검증합니다. 일치하지 않으면 scene을 생성하지 않으며, 충돌 검사를 활성화한 상태에서 연결된 scene이 누락되면 Guard가 실행을 차단합니다. 기존 노드에 남아 있는 scene 설정으로 대체하지 않습니다.

```powershell
python scripts/export_fanuc_usd_geometry.py C:/path/to/Test8.usd --output collision-inventory.json --proxy-output arm-proxy.json
```

[실제 장면의 결과](validation/fanuc-m710-usd-proxy-2026-10-20.json)는 USD/FK 대조 통과와 팔 6개·이동 베이스 2개·고정 레일 1개, 총 캡슐 9개 생성을 기록합니다. 현재 활성 충돌체의 매핑 누락은 없지만 툴·환경 전체 검증은 아직 남아 있습니다. `paddingM`은 추가 반경이며 기본값 0m입니다. 자기 충돌 제외 쌍은 자동 지정하지 않으므로 검토한 쌍만 `excludedSelfPairs`에 collider ID로 입력합니다. 보수적인 캡슐 중첩은 실제 mesh 접촉이 없어도 충돌로 보고될 수 있습니다.

현재 브리지는 새 명령을 거부합니다. 위 Script Editor 연결 후 다음 읽기 전용 명령으로 실제 스냅샷과 형상/FK 대조를 재확인합니다.

```powershell
python scripts/validate_fanuc_observation.py --endpoint ws://127.0.0.1:9910/omniverse --snapshot --geometry --output runtime-report.json
```

이 명령은 로봇 동작을 요청하지 않습니다. 현재 상태는 [실환경 검증 기록](validation/fanuc-m710-usd-proxy-runtime-2026-10-20.json)에 저장했습니다. 파일 기반 대조와 모델 생성은 완료했지만, 물리 동기화·전체 셀 충돌 검증·완료 관측까지 통과한 상태는 아닙니다.

## 시작 자세에서 첫 목표까지의 검사

기존 Execute 그래프에 `FANUC Physics Snapshot`을 추가하고 출력을 Preflight·Geometry Collision·Execution Guard의 `snapshot` 입력에 모두 연결합니다. Guard에서 `requireInitialSnapshot=true`로 설정하면 snapshot 미연결도 차단합니다. 연결된 snapshot 수집이 실패하면 오프라인 검사로 대체하지 않습니다.

검사는 시작 관절 위치 제한·축 집합·안정화 상태를 확인하고, 상대 시각 0초에서 첫 목표까지의 smoothstep 구간을 충돌 검사에 포함합니다. 실제 snapshot에 외부축이 있으면 명령에도 해당 축이 필요합니다. Preflight·충돌 보고서·Guard는 snapshot 전체 내용의 digest를 공유하며, 서로 다른 snapshot이나 오래된 관측을 거부합니다. Guard의 `maxSnapshotAgeSec` 기본값은 5초이고 브리지와 backend의 시스템 시각이 맞아야 합니다.

[합성 입력](../examples/graphs/fanuc_m710_initial_collision_example.json)은 첫 목표 이후에는 충돌이 없지만 시작 자세에서 첫 목표로 이동하는 1초 시점에 충돌합니다. 다음 명령은 해당 충돌을 검출해 종료 코드 1을 반환합니다.

```powershell
python scripts/validate_fanuc_collision.py examples/graphs/fanuc_m710_initial_collision_example.json --max-step-sec 0.1 --output initial-sweep-report.json
```

오프라인 입력은 `action`, `scene`, 선택 키 `snapshot`으로 구성합니다. 합성·저장 snapshot은 실시간 상태로 취급하지 않습니다. [결과 기록](validation/fanuc-m710-initial-sweep-synthetic-2026-10-20.json)과 [시작 구간 설계](vlagraph-autonomous-world-model-design-ko.md#시작-스냅샷에서-첫-목표까지의-검사)에 검증 범위를 기록했습니다. 물리 베이스·툴 pose 정렬, 전송 직전 실제 상태 재확인과 물리 rollout은 남아 있으며 기본 `full` 정책은 유지됩니다.

현재 브리지는 여전히 새 조회를 거부하므로 [실환경 재확인 기록](validation/fanuc-m710-start-state-runtime-2026-10-20.json)은 실패 상태입니다. Script Editor 어댑터 연결 후 실제 snapshot으로 재검증해야 합니다.

## 베이스·레일과 형상 누락 보고서

`FANUC USD Proxy`의 `coverage_report`를 Print에 연결하면 현재 포함된 형상과 누락 항목을 확인할 수 있습니다. `joint` 캡슐은 관절을, `base`는 외부축의 베이스 이동을 따르며, `world` 레일은 세계 좌표에 고정됩니다. 팔-베이스·팔-레일 자기 충돌도 기존 검사에 포함합니다.

고정 툴은 활성 FixedJoint의 연결과 변환을 검증할 때만 부모 링크에 포함합니다. 연결을 확인하지 못한 충돌체와 동적 장애물은 별도 목록으로 남기고, 포함된 형상에 충돌이 없어도 `indeterminate`로 처리해 Guard가 실행을 차단합니다. 실제 USD에 툴 형상이 빠져 있는 경우까지 자동 확인하지는 않으므로 `tool_coverage`, `environment_coverage`는 미검증으로 유지합니다.

[현재 모델의 진단 기록](validation/fanuc-m710-robot-proxy-diagnostic-2026-10-20.json)은 실제 USD에서 생성한 캡슐 9개에 합성 정지 궤적을 적용한 결과입니다. 캡슐 중첩으로 `collision`이며, 현재 로봇의 실제 물리 접촉 판정이 아닙니다. 자기 충돌 제외 쌍을 검토하고 툴·환경의 기대 형상을 확인해야 합니다. 상세 계약은 [베이스·레일 설계](vlagraph-autonomous-world-model-design-ko.md#베이스레일고정-부착물과-형상-누락-진단)에 기록했습니다.

## 자기 충돌 쌍과 툴·환경 기대 목록 검토

`FANUC USD Proxy.review_plan`을 Print에 연결해 검토 계획을 확인합니다. [현재 모델의 검토 파일](validation/fanuc-m710-geometry-review-plan-2026-10-20.json)은 36쌍 모두 `keep` 상태입니다. 합성 정지 궤적에서는 12쌍이 여유 거리 기준을 위반했고, 이 중 4쌍은 직접 관절 연결이 없어 더 세밀한 형상 검토가 필요합니다. 쌍별 집계는 상세 접촉 로그 100개 제한과 별개로 전체 표본을 사용합니다.

```powershell
python scripts/review_fanuc_geometry.py inventory.json proxy.json --collision-report collision-report.json --output review.json
```

실제 형상과 적용 자세 범위를 검토한 쌍만 `decision=exclude`로 바꾸고 `reviewed=true`, `reviewed_by`, `rationale`를 기록합니다. 인접 관절이라는 이유만으로 제외하지 않습니다. `geometry_expectations`의 tool·environment에 실제 필요한 prim 경로를 지정하고 `declared=true`로 설정합니다. 경로 아래에 활성 충돌 형상이 없거나 검사 모델에 포함되지 않으면 미확정으로 처리해 실행을 차단합니다.

```powershell
python scripts/review_fanuc_geometry.py inventory.json proxy.json --review review.json --output review-summary.json --scene-output reviewed-scene.json
```

그래프에서는 편집한 JSON을 Proxy의 `geometryReview` 설정 또는 선택 `review` 입력으로 전달합니다. 원본 inventory와 scene digest가 달라지면 검토 적용을 거부하므로 패딩·형상 변경 후에는 계획을 다시 생성해야 합니다. 기존 Execute 흐름은 유지합니다. 상세 범위는 [형상 검토 설계](vlagraph-autonomous-world-model-design-ko.md#자기-충돌-제외와-기대-형상-검토)를 참고합니다. 검토 파일 적용은 실제 물리 검증이나 실행 허가를 의미하지 않습니다.

## 비인접 4쌍의 세부 형상 검사

기록된 캡슐 최소 여유 거리 자세에서 Mesh의 선언된 `convexHull` 정점과 Cube 꼭짓점으로 거리 상·하한을 계산할 수 있습니다. 실제 USD의 합성 0도 자세에서는 3쌍이 약 35.6mm·141.4mm·162.1mm 분리됐고, 레일–베이스 1쌍은 접촉 또는 수치 허용 오차 안으로 나왔습니다. [세부 기록](validation/fanuc-m710-hull-inspection-2026-10-20.json)에 자세·형상 digest와 미검증 범위를 보존했습니다.

```powershell
python scripts/inspect_fanuc_hulls.py C:/path/to/Test8.usd inventory.json proxy.json diagnostic.json --output hull-inspection.json
python scripts/review_fanuc_geometry.py inventory.json proxy.json --collision-report diagnostic.json --hull-inspection hull-inspection.json --output review.json
```

`diagnostic.json`에는 검사에 사용한 `action`, `report`, 필요 시 `snapshot`을 포함합니다. 이전 단계에서 생성한 진단 wrapper를 사용할 수 있습니다. 검사에는 numpy·pxr가 필요하며 시뮬레이터나 로봇 동작을 요청하지 않습니다.

[검토 파일](validation/fanuc-m710-geometry-review-plan-2026-10-20.json)에 근거를 연결했으며 36쌍 모두 검사 유지 상태입니다. 지정 자세의 분리는 전체 이동 구간이나 PhysX cooked 형상의 무충돌을 의미하지 않습니다. 기존 Execute 가드와 제외 설정은 그대로 유지합니다. 상세 계약은 [선언된 볼록 형상 검사](vlagraph-autonomous-world-model-design-ko.md#비인접-쌍의-선언된-볼록-형상-검사)를 참고합니다.

## 선택된 볼록 형상 쌍의 구간 검사

`--sweep` 옵션으로 선택된 비인접 쌍을 궤적 구간에서 검사합니다. 관절 smoothstep을 표본화하고, 표본 사이의 최대 형상 이동 거리도 비교합니다. 표본에서 분리돼 있어도 구간 분리를 확인하지 못하면 `indeterminate`로 남깁니다. 지원하지 않는 형상도 미확정입니다.

```powershell
python scripts/inspect_fanuc_hulls.py C:/path/to/Test8.usd inventory.json proxy.json diagnostic.json --sweep --max-step-sec 0.05 --output hull-sweep.json
python scripts/review_fanuc_geometry.py inventory.json proxy.json --collision-report diagnostic.json --hull-inspection hull-inspection.json --hull-sweep hull-sweep.json --output review.json
```

[현재 기록](validation/fanuc-m710-hull-sweep-2026-10-20.json)은 실제 USD에 합성 정지 궤적을 적용한 2~4초·41개 표본 결과입니다. 선택된 4쌍 중 3쌍이 구간 분리를 통과했고 레일–베이스 1쌍은 여유 거리 미달입니다. 이는 실제 LS 이동·전체 충돌 쌍·PhysX 검증 완료를 뜻하지 않습니다. snapshot이 없으면 첫 목표 이전 구간도 미검증입니다.

결과는 [검토 파일](validation/fanuc-m710-geometry-review-plan-2026-10-20.json)의 `hull_sweep`에 연결했습니다. 36쌍 모두 검사 유지 상태이며 기본 Execute 가드는 그대로 적용됩니다. 세부 검사 보고서를 실행용 충돌 보고서 대신 연결하면 Guard가 거부합니다. 자세한 예산·이동 상한·상태 의미는 [볼록 형상 구간 검사 설계](vlagraph-autonomous-world-model-design-ko.md#선택된-볼록-형상-쌍의-이동-구간-검사)를 참고합니다.

## 실제 LS 구간별 검사와 기하 진단

기존 Execute 템플릿에 `M-710 LS 구간별 검사` 출력을 추가했습니다. Preflight의 `segment_report`에서 원본 행·P 번호·시각·이동 종류·termination과 구간 보류 원인을 확인합니다. 기존 저장 그래프는 새 출력을 Print에 연결하거나 템플릿을 다시 선택합니다. 구간 보고서는 부분 프로그램 실행용 action을 만들지 않습니다.

```powershell
python scripts/validate_fanuc_trajectory.py C:/path/to/program.LS --segment-output segments.json --collision-scene proxy.json --geometry-output ls-geometry.json --output preflight.json
```

전체 사전검사와 소스 명령 감사가 통과해야 preview action으로 기하 검사를 수행합니다. 실패 시 `ls-geometry.json`에는 보류 상태와 `action=null`, `report=null`을 저장합니다. 통과 시에는 기존 세부 형상 검사 CLI에서 사용할 수 있는 진단 wrapper를 저장합니다. snapshot이 없으면 첫 목표 이전은 미검증입니다.

실제 `S05PROC.LS`는 [73구간 모두 보류](validation/fanuc-m710-ls-segments-2026-10-20.json)됐고 [기하 검사도 보류](validation/fanuc-m710-ls-geometry-2026-10-20.json)됐습니다. UF/UT·L/CNT·첫 목표 시각 문제가 남아 있으며, WAIT 7개와 공정/IO 17개도 현재 실행 어댑터가 재현하지 않는 명령으로 표시했습니다. 이런 소스 명령이 남아 있으면 `static_simulation`에서도 Execute 가드가 차단합니다.

보정·시간 설정은 실제 확인한 값으로 기존 `--initial-settings`·`--settings`에 지정합니다. 검사 통과를 위해 원본 명령이나 오류 프레임을 제거하지 않습니다. 상세 범위와 상태 의미는 [실제 LS 연결 설계](vlagraph-autonomous-world-model-design-ko.md#실제-ls의-구간-검토와-기하-진단-연결)에 기록했습니다.
