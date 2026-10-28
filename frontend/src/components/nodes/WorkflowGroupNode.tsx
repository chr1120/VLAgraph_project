import React from "react";
import { Handle, Position, NodeProps } from "reactflow";

export default function WorkflowGroupNode({ data }: NodeProps) {
  return <div className={`workflow-group-card workflow-status-${data.status}`}>
    <Handle type="target" position={Position.Left} isConnectable={false} />
    <span className="workspace-eyebrow">FANUC WORKFLOW</span>
    <h3>{data.label}</h3>
    <p>{data.description}</p>
    <p>{data.memberIds.length}개 내부 노드 · {data.status === "error" ? "결과 확인 필요" : data.status === "running" ? "진행 중" : data.status === "success" ? "처리 완료" : "대기"}</p>
    <p className="workflow-group-detail">{data.detail}</p>
    <button type="button" className="secondary-btn nodrag nopan nowheel" onClick={(event) => {
      event.stopPropagation();
      data.onShowDetails();
    }}>상세 노드 보기</button>
    <Handle type="source" position={Position.Right} isConnectable={false} />
  </div>;
}
