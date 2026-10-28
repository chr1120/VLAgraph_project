import React from "react";
import { Handle, Position, NodeProps } from "reactflow";

export default function GraphBundleNode({ data }: NodeProps) {
  return <div className="graph-bundle-card">
    <Handle type="target" position={Position.Left} isConnectable={false} />
    <span className="workspace-eyebrow">GRAPH BUNDLE</span>
    <h3>{data.label}</h3>
    <p>{data.memberIds?.length || 0} nodes · {data.edgeCount || 0} connections</p>
    <p>Run Graph에 내부 노드가 포함됩니다.</p>
    <div className="nodrag nowheel bundle-card-actions">
      <button className="secondary-btn" onClick={data.onShowNodes}>Show Nodes</button>
      <button className="secondary-btn" onClick={data.onUnbundle} disabled={data.locked}>Unbundle</button>
    </div>
    <Handle type="source" position={Position.Right} isConnectable={false} />
  </div>;
}
