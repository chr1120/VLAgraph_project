import type { Node, Edge } from "reactflow";

// Derived presentation only: these cards never enter the saved/executed graph.
export function workflowView(nodes: Node[], edges: Edge[]) {
  const groups = new Map<string, Node[]>();
  for (const node of nodes) {
    if (!node.data?._workflowGroup || node.type === "graph_bundle") continue;
    const key = JSON.stringify([node.data._graphId || "main", node.data._workflowGroup]);
    groups.set(key, [...(groups.get(key) || []), node]);
  }
  const owners = new Map<string, string>();
  const cards: Node[] = [];
  for (const [key, members] of groups) {
    let id = `workflow-group:${key}`;
    while (nodes.some((node) => node.id === id) || cards.some((node) => node.id === id)) id += ":view";
    members.forEach((node) => owners.set(node.id, id));
    const statuses = members.map((node) => node.data.status);
    const status = statuses.some((value) => value === "error" || value === "warning") ? "error"
      : statuses.includes("running") ? "running"
      : statuses.every((value) => value === "ok" || value === "success") ? "success" : "idle";
    cards.push({ id, type: "workflow_group", position: {
      x: Math.min(...members.map((node) => node.position.x)), y: Math.min(...members.map((node) => node.position.y)),
    }, draggable: false, deletable: false, connectable: false, selectable: false,
    data: { _graphId: members[0].data._graphId || "main", label: members[0].data._workflowGroupTitle,
      description: members[0].data._workflowGroupDescription, status,
      memberIds: members.map((node) => node.id),
      detail: members.find((node) => node.data.status === "error")?.data.lastSummary || "내부 노드와 연결은 실행에 그대로 포함됩니다." } });
  }
  const viewEdges = new Map<string, Edge>();
  for (const edge of edges) {
    const source = owners.get(edge.source) || edge.source;
    const target = owners.get(edge.target) || edge.target;
    if (source === target) continue;
    if (source === edge.source && target === edge.target) { viewEdges.set(edge.id, edge); continue; }
    const key = `workflow-edge:${JSON.stringify([source, target])}`;
    const previous = viewEdges.get(key);
    const count = Number(previous?.data?.connectionCount || 0) + 1;
    viewEdges.set(key, { ...edge, id: key, source, target,
      sourceHandle: owners.has(edge.source) ? null : edge.sourceHandle,
      targetHandle: owners.has(edge.target) ? null : edge.targetHandle,
      deletable: false, selected: false,
      data: { connectionCount: count }, label: count > 1 ? `${count}개 연결` : undefined });
  }
  return { nodes: [...nodes.filter((node) => !owners.has(node.id)), ...cards], edges: [...viewEdges.values()] };
}
