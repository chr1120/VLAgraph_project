import type { Node, Edge } from "reactflow";

export const graphId = (node: Node) => String(node.data?._graphId || "main");
export const workspaceId = () => globalThis.crypto?.randomUUID?.() || `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
export type CanvasGraph = { id: string; label: string; description: string; nodeIds: string[]; nodeCount: number; edgeCount: number; bundled: boolean };

export function graphList(nodes: Node[], edges: Edge[]): CanvasGraph[] {
  const groups = new Map<string, CanvasGraph>();
  for (const node of nodes) {
    const id = graphId(node);
    const group = groups.get(id) || { id, label: String(node.data?._graphLabel || "Graph"),
      description: String(node.data?._graphDescription || ""), nodeIds: [], nodeCount: 0, edgeCount: 0, bundled: false };
    if (node.type === "graph_bundle") group.bundled = true;
    else { group.nodeIds.push(node.id); group.nodeCount += 1; }
    groups.set(id, group);
  }
  return [...groups.values()].filter((group) => group.nodeCount > 0).map((group) => {
    const ids = new Set(group.nodeIds);
    return { ...group, edgeCount: edges.filter((edge) => ids.has(edge.source) && ids.has(edge.target)).length };
  });
}

export function instantiateGraph(nodes: Node[], edges: Edge[], id: string, label: string, description: string, offsetX = 0) {
  const ids = new Map(nodes.map((node) => [node.id, `${id}:${node.id}`]));
  const minX = nodes.length ? Math.min(...nodes.map((node) => node.position.x)) : 0;
  return {
    nodes: nodes.map((node) => ({ ...node, id: ids.get(node.id)!, selected: false,
      position: { ...node.position, x: node.position.x - minX + offsetX },
      data: { ...node.data, _graphId: id, _graphLabel: label, _graphDescription: description } })),
    edges: edges.map((edge) => ({ ...edge, id: `${id}:${edge.id}`, source: ids.get(edge.source)!, target: ids.get(edge.target)!, selected: false })),
  };
}

// Bundle cards only change the view. Actual nodes, handles and edges stay intact.
export function bundleGraph(nodes: Node[], edges: Edge[], selectedGraphId: string, id: string): Node {
  const graph = graphList(nodes, edges).find((group) => group.id === selectedGraphId);
  if (!graph) throw new Error("먼저 Graphs 탭에서 그래프를 선택하세요.");
  if (graph.bundled) throw new Error("이미 번들로 묶인 그래프입니다.");
  if (nodes.some((node) => node.id === id)) throw new Error("Duplicate bundle ID");
  const members = nodes.filter((node) => graph.nodeIds.includes(node.id));
  return { id, type: "graph_bundle", deletable: false, connectable: false,
    position: { x: Math.min(...members.map((node) => node.position.x)), y: Math.min(...members.map((node) => node.position.y)) },
    data: { label: `${graph.label} Bundle`, _graphId: graph.id, _graphLabel: graph.label,
      _graphDescription: graph.description, memberIds: graph.nodeIds, edgeCount: graph.edgeCount } };
}

export function executionGraph(nodes: Node[], edges: Edge[]) {
  if (nodes.some((node) => node.type === "workflow_group")) throw new Error("표시용 그룹 대신 실제 내부 노드로 실행해야 합니다.");
  const actualNodes = nodes.filter((node) => node.type !== "graph_bundle");
  const ids = new Set(actualNodes.map((node) => node.id));
  if (edges.some((edge) => !ids.has(edge.source) || !ids.has(edge.target))) throw new Error("실제 노드가 없는 연결이 있습니다. 그래프 연결을 확인하세요.");
  return { nodes: actualNodes, edges };
}

export function bundleView(nodes: Node[], edges: Edge[]) {
  const owner = new Map<string, string>();
  nodes.filter((node) => node.type === "graph_bundle").forEach((bundle) => {
    (bundle.data.memberIds as string[]).forEach((id) => owner.set(id, bundle.id));
  });
  return {
    nodes: nodes.filter((node) => !owner.has(node.id)),
    edges: edges.flatMap((edge) => {
      const source = owner.get(edge.source) || edge.source;
      const target = owner.get(edge.target) || edge.target;
      if (source === target) return [];
      if (source === edge.source && target === edge.target) return [edge];
      return [{ ...edge, id: `bundle-view:${edge.id}`, source, target,
        sourceHandle: owner.has(edge.source) ? null : edge.sourceHandle,
        targetHandle: owner.has(edge.target) ? null : edge.targetHandle,
        deletable: false, selectable: false, selected: false, style: { ...edge.style, strokeDasharray: "5 4" } }];
    }),
  };
}

export function serializeWorkspace(nodes: Node[], edges: Edge[]) {
  return JSON.stringify({ schema: "vlagraph.workspace.v1",
    nodes: nodes.map(({ id, type, position, data }) => ({ id, type, position, data })),
    edges: edges.map(({ selected, ...edge }) => edge) },
    (_key, value) => typeof value === "function" ? undefined : value, 2);
}

export function parseWorkspace(text: string, supportedTypes: string[]): { nodes: Node[]; edges: Edge[] } {
  const graph = JSON.parse(text.replace(/^\uFEFF/, ""));
  if (!graph || !Array.isArray(graph.nodes) || !Array.isArray(graph.edges)) throw new Error("nodes/edges 배열이 필요합니다.");
  if (graph.schema && graph.schema !== "vlagraph.workspace.v1") throw new Error("지원하지 않는 그래프 형식입니다.");
  const ids = new Set<string>();
  const nodes: Node[] = graph.nodes.map((node: Node) => {
    if (!node || typeof node.id !== "string" || !node.id || ids.has(node.id)) throw new Error("잘못되거나 중복된 노드 ID입니다.");
    if (node.type === "workflow_group") throw new Error("표시용 그룹 대신 실제 내부 노드를 불러와야 합니다.");
    if (!supportedTypes.includes(node.type || "")) throw new Error(`지원하지 않는 노드: ${node.type}`);
    if (!Number.isFinite(node.position?.x) || !Number.isFinite(node.position?.y) || !node.data || typeof node.data !== "object" || Array.isArray(node.data)) throw new Error("노드 위치/설정이 올바르지 않습니다.");
    ids.add(node.id);
    return { id: node.id, type: node.type, position: node.position, data: node.data,
      ...(node.type === "graph_bundle" ? { deletable: false, connectable: false } : {}) };
  });
  const byId = new Map(nodes.map((node) => [node.id, node]));
  const edgeIds = new Set<string>();
  const edges: Edge[] = graph.edges.map((edge: Edge) => {
    if (!edge || typeof edge.id !== "string" || !edge.id || edgeIds.has(edge.id) || !ids.has(edge.source) || !ids.has(edge.target)) throw new Error("연결 ID/대상 노드를 확인하세요.");
    if (byId.get(edge.source)?.type === "graph_bundle" || byId.get(edge.target)?.type === "graph_bundle") throw new Error("번들 카드 대신 실제 노드의 핀에 연결해야 합니다.");
    edgeIds.add(edge.id);
    return { ...edge, selected: false };
  });
  const owned = new Set<string>();
  const bundledGraphs = new Set<string>();
  nodes.filter((node) => node.type === "graph_bundle").forEach((bundle) => {
    const members = bundle.data.memberIds;
    if (!Array.isArray(members) || !members.length || bundledGraphs.has(graphId(bundle))) throw new Error("빈 번들이거나 중복 번들입니다.");
    bundledGraphs.add(graphId(bundle));
    const expected = nodes.filter((node) => node.type !== "graph_bundle" && graphId(node) === graphId(bundle));
    if (expected.length !== members.length) throw new Error("번들에 포함된 그래프 노드 수가 다릅니다.");
    members.forEach((id: string) => {
      const member = byId.get(id);
      if (!member || member.type === "graph_bundle" || owned.has(id) || graphId(member) !== graphId(bundle)) throw new Error("번들 멤버/그래프 구성이 올바르지 않습니다.");
      owned.add(id);
    });
  });
  return { nodes, edges };
}
