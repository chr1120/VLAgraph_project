import asyncio
import threading
import time

import pytest

from backend.executors import amr_nodes
from backend.executors.registry import NODE_EXECUTORS
from backend.runtime.session_manager import GraphRuntimeManager


def graph(preview=False):
    return {"nodes": [{"id": "navigate", "type": "amr_navigate", "data": {
        "enableExecution": not preview, "omniverseEndpoint": "ws://simulator:9900/omniverse",
        "navigationTimeoutSec": 1}}], "edges": []}


async def until(predicate):
    deadline = time.monotonic()+3
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Timed out waiting for runtime state")
        await asyncio.sleep(.005)


@pytest.fixture
def transport(monkeypatch):
    states, calls, submitted = {}, [], threading.Event()
    def record(command_id):
        return {"command_id": command_id, "status": states[command_id],
                "completion_verified": states[command_id] == "completed"}
    def request(props, command, **fields):
        command_id = fields["command_id"]
        calls.append((command, command_id, props.get("omniverseEndpoint")))
        if command_id not in states:
            return {"ok": False, "message": "Unknown command_id"}
        if command == "amr_stop":
            states[command_id] = "cancelled"
        return {"ok": True, "execution": record(command_id)}
    def navigate(node, context, edges):
        if node["data"].get("enableExecution") is not True:
            return {"ok": True, "started": False, "execution": None}
        command_id = node["data"]["commandId"]
        states[command_id] = "running"
        submitted.set()
        return {"ok": True, "started": True, "command_id": command_id, "execution": record(command_id)}
    monkeypatch.setattr(amr_nodes, "bridge_request", request)
    monkeypatch.setitem(NODE_EXECUTORS, "amr_navigate", navigate)
    return states, calls, submitted, record


def test_session_waits_for_actual_amr_arrival_before_completing(transport):
    states, calls, submitted, _ = transport
    async def scenario():
        manager = GraphRuntimeManager()
        snapshot = await manager.start_session(graph())
        session_id = snapshot["session_id"]
        task = manager._sessions[session_id]["task"]
        await until(lambda: submitted.is_set() and bool(calls))
        assert manager.get_session(session_id)["status"] not in {"completed", "error", "cancelled"}
        command_id = next(iter(states))
        states[command_id] = "completed"
        await asyncio.wait_for(task, 3)
        snapshot = manager.get_session(session_id)
        assert snapshot["status"] == "completed"
        assert snapshot["amr_commands"][0]["completion_verified"] is True
        assert not any(command == "amr_stop" for command, _, _ in calls)
    asyncio.run(scenario())


def test_cancel_sends_stop_to_the_owned_command_and_endpoint(transport):
    states, calls, submitted, _ = transport
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(graph()))["session_id"]
        task = manager._sessions[session_id]["task"]
        await until(submitted.is_set)
        await manager.cancel_session(session_id)
        await asyncio.wait_for(task, 3)
        assert manager.get_session(session_id)["status"] == "cancelled"
        assert list(states.values()) == ["cancelled"]
        assert ("amr_stop", next(iter(states)), "ws://simulator:9900/omniverse") in calls
    asyncio.run(scenario())


def test_cancel_before_navigate_ack_retries_stop_after_late_submission(transport, monkeypatch):
    states, calls, _, record = transport
    sending, release = threading.Event(), threading.Event()
    def delayed(node, context, edges):
        sending.set()
        assert release.wait(3)
        command_id = node["data"]["commandId"]
        states[command_id] = "running"
        return {"ok": True, "command_id": command_id, "execution": record(command_id)}
    monkeypatch.setitem(NODE_EXECUTORS, "amr_navigate", delayed)
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(graph()))["session_id"]
        task = manager._sessions[session_id]["task"]
        try:
            await until(sending.is_set)
            await manager.cancel_session(session_id)  # First stop sees unknown ID.
        finally:
            release.set()
        await asyncio.wait_for(task, 3)
        assert list(states.values()) == ["cancelled"]
        assert sum(command == "amr_stop" for command, _, _ in calls) >= 2
        assert manager.get_session(session_id)["status"] == "cancelled"
    asyncio.run(scenario())


def test_graph_error_after_submission_stops_amr(transport, monkeypatch):
    states, calls, _, _ = transport
    monkeypatch.setitem(NODE_EXECUTORS, "print", lambda *a: {"ok": False, "message": "downstream failed"})
    failed = graph()
    failed["nodes"].append({"id": "failed", "type": "print", "data": {}})
    failed["edges"].append({"source": "navigate", "target": "failed"})
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(failed))["session_id"]
        await asyncio.wait_for(manager._sessions[session_id]["task"], 3)
        assert manager.get_session(session_id)["status"] == "error"
        assert list(states.values()) == ["cancelled"]
        assert any(command == "amr_stop" for command, _, _ in calls)
    asyncio.run(scenario())


def test_preview_creates_no_motion_ownership_or_status_requests(transport):
    states, calls, _, _ = transport
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(graph(preview=True)))["session_id"]
        await asyncio.wait_for(manager._sessions[session_id]["task"], 3)
        assert manager.get_session(session_id)["status"] == "completed"
        assert not states and not calls
    asyncio.run(scenario())


def test_unconfirmed_stop_is_reported_as_error_with_command_id(transport, monkeypatch):
    states, calls, submitted, _ = transport
    original = amr_nodes.bridge_request
    def disconnected(props, command, **fields):
        if command == "amr_stop":
            raise OSError("simulator disconnected")
        return original(props, command, **fields)
    monkeypatch.setattr(amr_nodes, "bridge_request", disconnected)
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(graph()))["session_id"]
        task = manager._sessions[session_id]["task"]
        await until(submitted.is_set)
        await manager.cancel_session(session_id)
        await asyncio.wait_for(task, 3)
        snapshot = manager.get_session(session_id)
        assert snapshot["status"] == "error"
        assert next(iter(states)) in snapshot["error"]
        assert "stop unconfirmed" in snapshot["error"]
    asyncio.run(scenario())


def test_repeated_cycles_use_new_automatic_ids_after_verified_completion(transport, monkeypatch):
    states, calls, _, record = transport
    issued = []
    def already_done(node, context, edges):
        command_id = node["data"]["commandId"]
        issued.append(command_id)
        states[command_id] = "completed"
        return {"ok": True, "command_id": command_id, "execution": record(command_id)}
    monkeypatch.setitem(NODE_EXECUTORS, "amr_navigate", already_done)
    async def scenario():
        manager = GraphRuntimeManager()
        session_id = (await manager.start_session(graph(), options={
            "persistent": True, "require_feedback": False, "max_iterations": 2, "cycle_interval_sec": .01
        }))["session_id"]
        await asyncio.wait_for(manager._sessions[session_id]["task"], 3)
        assert manager.get_session(session_id)["status"] == "completed"
        assert len(issued) == 2 and len(set(issued)) == 2
    asyncio.run(scenario())
