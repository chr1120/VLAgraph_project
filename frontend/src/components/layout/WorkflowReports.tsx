import React, { useState } from "react";
import type { Node } from "reactflow";

function Report({ node, value }: { node: Node; value: unknown }) {
  const [open, setOpen] = useState(false);
  return <details className="workflow-node-report" onToggle={(event) => setOpen(event.currentTarget.open)}>
    <summary>{node.data.label || node.type}</summary>
    {open && (value === undefined ? <p>아직 실행 결과가 없습니다.</p> : <>
      <button className="secondary-btn" onClick={() => {
        const url = URL.createObjectURL(new Blob([JSON.stringify(value, null, 2)], { type: "application/json" }));
        const link = document.createElement("a"); link.href = url; link.download = `${node.type}-report.json`; link.click();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
      }}>전체 JSON 저장</button>
      <pre>{JSON.stringify(value, null, 2).slice(0, 100000)}</pre>
      {JSON.stringify(value).length > 100000 && <p>화면에는 일부만 표시합니다. 전체 JSON을 저장해 확인하세요.</p>}
    </>)}
  </details>;
}

export default function WorkflowReports({ nodes, results }: { nodes: Node[]; results: Record<string, unknown> }) {
  const stages = new Map<string, Node[]>();
  for (const node of nodes) {
    if (!node.data._workflowStage) continue;
    const stage = String(node.data._workflowStage);
    stages.set(stage, [...(stages.get(stage) || []), node]);
  }
  if (!stages.size) return null;
  return <section className="workflow-reports">
    <h3>FANUC 단계별 보고서</h3>
    <p>표시 단계를 펼치면 각 검사의 실제 결과를 확인할 수 있습니다.</p>
    {[...stages.entries()].sort(([a], [b]) => a.localeCompare(b)).map(([stage, members]) =>
      <details key={stage} className="workflow-stage-report">
        <summary>{stage} · {members.filter((node) => results[node.id] !== undefined).length}/{members.length}개 결과</summary>
        {members.map((node) => <Report key={node.id} node={node} value={results[node.id]} />)}
      </details>)}
  </section>;
}
