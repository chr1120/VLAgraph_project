# backend/runtime/runner.py

from __future__ import annotations

import asyncio
import inspect
import time
from typing import Any, Callable, Dict, List
from fastapi import HTTPException

from backend.executors.registry import NODE_EXECUTORS


class GraphRunCancelled(Exception):
    """Raised when a long-running graph session is cancelled between nodes."""


NodeResultCallback = Callable[[Dict[str, Any]], None]


def summarize_for_log(value: Any, depth: int = 0) -> Any:
    if isinstance(value, str):
        compact = " ".join(value.split())
        if len(compact) > 140:
            return f"{compact[:137]}..."
        return compact

    if depth >= 2:
        if isinstance(value, dict):
            return "{...}"
        if isinstance(value, list):
            return "[...]"
        return value

    if isinstance(value, dict):
        items = list(value.items())
        summarized: Dict[str, Any] = {}
        for key, item in items[:6]:
            summarized[str(key)] = summarize_for_log(item, depth + 1)
        if len(items) > 6:
            summarized["..."] = f"{len(items) - 6} more fields"
        return summarized

    if isinstance(value, list):
        summarized = [summarize_for_log(item, depth + 1) for item in value[:6]]
        if len(value) > 6:
            summarized.append(f"... {len(value) - 6} more items")
        return summarized

    return value


def build_node_map(nodes: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {node["id"]: node for node in nodes}


def topo_sort(nodes: List[Dict[str, Any]], edges: List[Dict[str, Any]]) -> List[str]:
    indegree = {node["id"]: 0 for node in nodes}
    graph: Dict[str, List[str]] = {node["id"]: [] for node in nodes}

    for edge in edges:
        source = edge["source"]
        target = edge["target"]

        if source not in graph:
            raise HTTPException(status_code=400, detail=f"Unknown source node: {source}")
        if target not in indegree:
            raise HTTPException(status_code=400, detail=f"Unknown target node: {target}")

        graph[source].append(target)
        indegree[target] += 1

    queue = [node_id for node_id, degree in indegree.items() if degree == 0]
    ordered: List[str] = []

    while queue:
        current = queue.pop(0)
        ordered.append(current)

        for nxt in graph[current]:
            indegree[nxt] -= 1
            if indegree[nxt] == 0:
                queue.append(nxt)

    if len(ordered) != len(nodes):
        raise HTTPException(status_code=400, detail="Graph contains a cycle")

    return ordered


def execute_node(
    executor: Any,
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
    stop_requested: Callable[[], bool] | None = None,
) -> Any:
    sig = None
    try:
        sig = inspect.signature(executor)
        param_count = len(sig.parameters)
    except Exception:
        param_count = 3

    if "stop_requested" in getattr(sig, "parameters", {}):
        result = executor(node, context, edges, stop_requested=stop_requested)
    elif param_count >= 3:
        result = executor(node, context, edges)
    else:
        result = executor(node, context)

    if inspect.isawaitable(result):
        result = asyncio.run(result)

    return result


def run_graph(
    graph: Dict[str, Any],
    input_data: Dict[str, Any] | None = None,
    *,
    on_node_result: NodeResultCallback | None = None,
    on_node_start: NodeResultCallback | None = None,
    stop_requested: Callable[[], bool] | None = None,
    initial_context: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    nodes: List[Dict[str, Any]] = graph.get("nodes", [])
    edges: List[Dict[str, Any]] = graph.get("edges", [])

    node_map = build_node_map(nodes)
    ordered_ids = topo_sort(nodes, edges)

    context: Dict[str, Any] = initial_context if initial_context is not None else {}
    logs: List[str] = []

    if input_data:
        context["_input"] = input_data

    for node_id in ordered_ids:
        if stop_requested and stop_requested():
            raise GraphRunCancelled("Graph run cancelled.")

        node = node_map[node_id]
        node_type = node.get("type")

        if node_type not in NODE_EXECUTORS:
            raise HTTPException(status_code=400, detail=f"Unknown node type: {node_type}")

        executor = NODE_EXECUTORS[node_type]
        if on_node_start:
            on_node_start({"node_id": node_id, "node_type": node_type, "node": node})
        started_at = time.time()

        try:
            result = execute_node(executor, node, context, edges, stop_requested=stop_requested)
        except Exception as e:
            context[node_id] = {
                "error": str(e),
                "ok": False,
            }
            log_line = f"[ERROR] {node_id} ({node_type}) -> {str(e)}"
            logs.append(log_line)
            if on_node_result:
                on_node_result(
                    {
                        "node_id": node_id,
                        "node_type": node_type,
                        "ok": False,
                        "result": context[node_id],
                        "log_line": log_line,
                        "duration_ms": int((time.time() - started_at) * 1000),
                    }
                )
            continue

        context[node_id] = result
        log_line = f"[OK] {node_id} ({node_type}) -> {summarize_for_log(result)}"
        logs.append(log_line)
        if on_node_result:
            on_node_result(
                {
                    "node_id": node_id,
                    "node_type": node_type,
                    "ok": True,
                    "result": result,
                    "log_line": log_line,
                    "duration_ms": int((time.time() - started_at) * 1000),
                }
            )

    return {
        "ok": True,
        "logs": logs,
        "context": context,
        "order": ordered_ids,
        "error_count": sum(
            1
            for node_id in ordered_ids
            if isinstance(context.get(node_id), dict)
            and (
                context[node_id].get("ok") is False
                or bool(context[node_id].get("error"))
            )
        ),
    }
