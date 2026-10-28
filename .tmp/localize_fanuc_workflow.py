import json
from pathlib import Path

labels = {
    "1. Robot Setup": "1. 로봇 설정",
    "2. LS Input": "2. LS 입력",
    "3. Prepare Motion": "3. 명령 준비",
    "4. Validation": "4. 통합 검증",
    "4. Trajectory Preflight": "4. 궤적 사전검사",
    "5. Execution Request": "5. 실행 요청",
    "6. Completion Check": "6. 완료 확인",
    "Parse LS and prepare joints / IK": "LS 파싱 · 초기 설정 · 관절·IK 명령 준비",
    "Snapshot, scene, geometry, context, collision and Guard": "현재 상태 · 장면·USD · 충돌 · 기억 대조 · Guard",
}
for name in ("fanuc-m710.json", "fanuc-m710-preflight.json"):
    path = Path("frontend/src/templates") / name
    value = json.loads(path.read_text(encoding="utf-8"))
    for node in value["nodes"]:
        node["data"] = {key: labels.get(item, item) if isinstance(item, str) else item
                        for key, item in node["data"].items()}
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
