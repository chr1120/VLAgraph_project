import { readFileSync } from "node:fs";
import assert from "node:assert/strict";
import { test } from "node:test";
import ts from "typescript";

const source = readFileSync(new URL("../src/graphWorkspace.ts", import.meta.url), "utf8");
const { outputText } = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2020 } });
const { graphList, instantiateGraph, bundleGraph, bundleView, executionGraph, parseWorkspace, serializeWorkspace } = await import(`data:text/javascript;base64,${Buffer.from(outputText).toString("base64")}`);
const node = (id, group = "main") => ({ id, type: "prompt", position: { x: 10, y: 20 }, data: { _graphId: group, _graphLabel: group, label: id } });
const nodes = [node("config"), node("guard"), node("execute", "second")];
const edges = [
  { id: "internal", source: "config", target: "guard", sourceHandle: "robot_config", targetHandle: "robot_config" },
  { id: "boundary", source: "guard", target: "execute", sourceHandle: "approval", targetHandle: "guard" },
];
const types = ["prompt", "graph_bundle"];

test("Graphs list counts internal edges, graph membership and bundle state", () => {
  const groups = graphList(nodes, edges);
  assert.equal(groups[0].nodeCount, 2); assert.equal(groups[0].edgeCount, 1);
  assert.equal(groups[1].nodeCount, 1); assert.equal(groups[1].edgeCount, 0);
  const bundle = bundleGraph(nodes, edges, "main", "bundle");
  assert.equal(graphList([...nodes, bundle], edges)[0].bundled, true);
});

test("adding two copies remaps every edge and preserves pins, robot settings and separate membership", () => {
  const first = instantiateGraph(nodes, edges, "first", "FANUC", "Example", 500);
  const second = instantiateGraph(nodes, edges, "second", "FANUC", "Example", 1000);
  const ids = [...first.nodes, ...second.nodes].map((n) => n.id);
  assert.equal(new Set(ids).size, nodes.length * 2);
  assert.equal(first.edges[0].source, "first:config");
  assert.equal(first.edges[0].targetHandle, "robot_config");
  assert.equal(first.nodes[0].position.x, 500);
  assert.equal(graphList([...first.nodes, ...second.nodes], [...first.edges, ...second.edges]).length, 2);
});

test("bundle selected graph preserves original execution nodes and external connections", () => {
  const bundle = bundleGraph(nodes, edges, "main", "bundle");
  assert.deepEqual(bundle.data.memberIds, ["config", "guard"]);
  const all = [...nodes, bundle];
  assert.deepEqual(executionGraph(all, edges), { nodes, edges });
  const view = bundleView(all, edges);
  assert.deepEqual(view.nodes.map((n) => n.id), ["execute", "bundle"]);
  assert.equal(view.edges.length, 1);
  assert.equal(view.edges[0].source, "bundle");
  assert.equal(view.edges[0].targetHandle, "guard");
  assert.equal(view.edges[0].deletable, false);
  assert.equal(edges[1].source, "guard");
});

test("two bundles preserve their boundary link and execute each original node once", () => {
  const all = [...nodes, bundleGraph(nodes, edges, "main", "b1"), bundleGraph(nodes, edges, "second", "b2")];
  const view = bundleView(all, edges);
  assert.equal(view.edges[0].source, "b1"); assert.equal(view.edges[0].target, "b2");
  assert.deepEqual(executionGraph(all, edges), { nodes, edges });
});

test("unbundle restores original nodes and edges", () => {
  assert.deepEqual(bundleView(nodes, edges), { nodes, edges });
});

test("JSON round-trip retains settings and nested dimensions but removes UI callbacks and transient node fields", () => {
  const custom = nodes.map((n) => ({ ...n, hidden: true, data: { ...n.data, sensor: { width: 640, height: 480 }, onRemove() {} } }));
  const all = [...custom, bundleGraph(custom, edges, "main", "b1")];
  const loaded = parseWorkspace(serializeWorkspace(all, edges), types);
  assert.equal(loaded.nodes[0].data.sensor.width, 640);
  assert.equal(loaded.nodes[0].data.onRemove, undefined);
  assert.equal(loaded.nodes[0].hidden, undefined);
  assert.deepEqual(executionGraph(loaded.nodes, loaded.edges).edges.map(({ selected, ...e }) => e), edges);
  assert.equal(loaded.nodes.at(-1).deletable, false);
});

test("rejects duplicate bundles and invalid selection", () => {
  assert.throws(() => bundleGraph(nodes, edges, "absent", "b"));
  const bundle = bundleGraph(nodes, edges, "main", "b");
  assert.throws(() => bundleGraph([...nodes, bundle], edges, "main", "b2"));
});

test("rejects malformed node and edge imports", () => {
  for (const broken of [
    { nodes: [node("a"), node("a")], edges: [] },
    { nodes, edges: [{ id: "bad", source: "config", target: "absent" }] },
    { nodes: [{ ...node("bad"), type: "unknown" }], edges: [] },
    { nodes: [{ ...node("bad"), position: { x: null, y: 2 } }], edges: [] },
    { nodes, edges, schema: "unsupported" },
  ]) assert.throws(() => parseWorkspace(JSON.stringify(broken), types));
});

test("rejects nested, overlapping, incomplete or wrong-graph bundles and bundle endpoint edges", () => {
  const bundle = bundleGraph(nodes, edges, "main", "b");
  assert.throws(() => parseWorkspace(JSON.stringify({ nodes: [...nodes, bundle, { ...bundle, id: "b2" }], edges }), types));
  for (const memberIds of [["b"], ["config"], ["config", "execute"], ["config", "config"]]) {
    assert.throws(() => parseWorkspace(JSON.stringify({ nodes: [...nodes, { ...bundle, data: { ...bundle.data, memberIds } }], edges }), types));
  }
  assert.throws(() => parseWorkspace(JSON.stringify({ nodes: [...nodes, bundle], edges: [{ id: "bad", source: "b", target: "execute" }] }), types));
});

test("execution refuses dangling edges rather than silently dropping them", () => {
  assert.throws(() => executionGraph(nodes, [...edges, { id: "bad", source: "config", target: "missing" }]));
});

test("actual FANUC robot config, guard and trajectory data survive add, bundle, save and load", () => {
  const graph = JSON.parse(readFileSync(new URL("../../examples/graphs/isaacsim_fanuc_m710ic20l_graph.json", import.meta.url), "utf8"));
  const added = instantiateGraph(graph.nodes, graph.edges, "fanuc", "FANUC", "Robot test", 500);
  const bundle = bundleGraph(added.nodes, added.edges, "fanuc", "bundle-fanuc");
  const loaded = parseWorkspace(serializeWorkspace([...added.nodes, bundle], added.edges), [...new Set(graph.nodes.map((n) => n.type)), "graph_bundle"]);
  const execution = executionGraph(loaded.nodes, loaded.edges);
  assert.equal(execution.nodes.length, graph.nodes.length);
  assert.deepEqual(execution.nodes.map((n) => n.data), added.nodes.map((n) => n.data));
  assert.deepEqual(execution.edges, added.edges);
  assert.ok(execution.nodes.some((n) => n.type === "robot_setup"));
  assert.ok(execution.nodes.some((n) => n.type === "execution_guard"));
});

test("bundled Prompt/Print graph runs against live backend", { skip: !process.env.GRAPH_API_BASE }, async () => {
  const plain = [
    { ...node("input"), data: { text: "bundle-smoke-test" } },
    { ...node("output"), type: "print", data: {} },
  ];
  const connections = [{ id: "text", source: "input", target: "output", sourceHandle: "text", targetHandle: "input" }];
  const bundle = bundleGraph(plain, connections, "main", "bundle");
  const response = await fetch(`${process.env.GRAPH_API_BASE}/api/graph/run`, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(executionGraph([...plain, bundle], connections)), signal: AbortSignal.timeout(15000),
  });
  assert.equal(response.status, 200);
  const result = await response.json();
  assert.equal(result.error_count, 0);
  assert.deepEqual(result.order, ["input", "output"]);
  assert.equal(result.context.input.text, "bundle-smoke-test");
  assert.equal(result.context.output.printed, "bundle-smoke-test");
});
