import { readFileSync } from "node:fs";
import assert from "node:assert/strict";
import { test } from "node:test";
import ts from "typescript";

async function sourceModule(name) {
  const source = readFileSync(new URL(`../src/${name}.ts`, import.meta.url), "utf8");
  const { outputText } = ts.transpileModule(source, { compilerOptions: { module: ts.ModuleKind.ESNext, target: ts.ScriptTarget.ES2020 } });
  return import(`data:text/javascript;base64,${Buffer.from(outputText).toString("base64")}`);
}
const { workflowView } = await sourceModule("workflowView");
const { instantiateGraph, executionGraph, bundleGraph, bundleView, parseWorkspace, serializeWorkspace } = await sourceModule("graphWorkspace");
const graph = JSON.parse(readFileSync(new URL("../src/templates/fanuc-m710.json", import.meta.url), "utf8"));

test("FANUC shows six stages while retaining all fourteen execution nodes and full Guard", () => {
  const original = structuredClone(graph);
  const view = workflowView(graph.nodes, graph.edges);
  assert.equal(view.nodes.length, 6);
  assert.equal(graph.nodes.length, 14);
  assert.equal(graph.nodes.filter((node) => node.type === "print").length, 0);
  assert.equal(view.nodes.filter((node) => node.type === "workflow_group").length, 2);
  const validation = view.nodes.find((node) => node.data.memberIds?.length === 8);
  assert.ok(validation);
  assert.equal(graph.nodes.find((node) => node.type === "execution_guard").data.validationPolicy, "full");
  assert.equal(graph.nodes.find((node) => node.type === "execution_guard").data.requireWorldContext, true);
  assert.deepEqual(executionGraph(graph.nodes, graph.edges), graph);
  assert.deepEqual(graph, original);
  assert.ok(view.edges.length < graph.edges.length);
});

test("two FANUC instances keep separate collapsed groups and every original execution connection", () => {
  const first = instantiateGraph(graph.nodes, graph.edges, "arm1", "Arm 1", "", 0);
  const second = instantiateGraph(graph.nodes, graph.edges, "arm2", "Arm 2", "", 3000);
  const nodes = [...first.nodes, ...second.nodes], edges = [...first.edges, ...second.edges];
  const view = workflowView(nodes, edges);
  assert.equal(view.nodes.length, 12);
  assert.equal(new Set(view.nodes.map((node) => node.id)).size, 12);
  assert.equal(executionGraph(nodes, edges).nodes.length, 28);
  assert.deepEqual(executionGraph(nodes, edges).edges, edges);
  for (const card of view.nodes.filter((node) => node.type === "workflow_group")) {
    assert.ok(card.data.memberIds.every((id) => id.startsWith(`${card.data._graphId}:`)));
  }
});

test("save/load retains fold metadata and refuses to execute or import virtual cards", () => {
  const types = [...new Set(graph.nodes.map((node) => node.type)), "workflow_group"];
  const loaded = parseWorkspace(serializeWorkspace(graph.nodes, graph.edges), types);
  assert.equal(workflowView(loaded.nodes, loaded.edges).nodes.length, 6);
  const view = workflowView(loaded.nodes, loaded.edges);
  assert.throws(() => executionGraph(view.nodes, view.edges));
  assert.throws(() => parseWorkspace(JSON.stringify(view), types));
});

test("whole graph bundles still contain all actual checks and completion observation", () => {
  const nodes = instantiateGraph(graph.nodes, graph.edges, "arm", "Arm", "").nodes;
  const edges = instantiateGraph(graph.nodes, graph.edges, "arm", "Arm", "").edges;
  const bundle = bundleGraph(nodes, edges, "arm", "bundle");
  const bundled = bundleView([...nodes, bundle], edges);
  assert.deepEqual(workflowView(bundled.nodes, bundled.edges).nodes.map((node) => node.id), ["bundle"]);
  assert.equal(executionGraph([...nodes, bundle], edges).nodes.length, 14);
});

test("completion observes the actual execution result and uses the same robot config", () => {
  const observe = graph.nodes.find((node) => node.type === "execution_observe");
  assert.equal(observe.data.waitForCompletion, true);
  assert.ok(graph.edges.some((edge) => edge.target === observe.id && edge.targetHandle === "result" && edge.source === "fanuc-m710-execute"));
  assert.ok(graph.edges.some((edge) => edge.target === observe.id && edge.targetHandle === "robot_config"));
});

test("all FANUC example exports exactly match the frontend canonical JSON", () => {
  const names = { "fanuc-m710": "isaacsim_fanuc_m710ic20l_graph", "fanuc-m710-preflight": "fanuc_m710_preflight_graph",
    "fanuc-m710-snapshot": "fanuc_m710_snapshot_graph", "fanuc-m710-observe": "fanuc_m710_world_observe_graph" };
  for (const [source, exported] of Object.entries(names)) {
    assert.equal(readFileSync(new URL(`../src/templates/${source}.json`, import.meta.url), "utf8"),
      readFileSync(new URL(`../../examples/graphs/${exported}.json`, import.meta.url), "utf8"));
  }
  const page = readFileSync(new URL("../src/pages/GraphPage.tsx", import.meta.url), "utf8");
  assert.ok(page.includes('import fanucM710Graph from "../templates/fanuc-m710.json"'));
});
