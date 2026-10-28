// pages/GraphPage.tsx

import React, { useCallback, useEffect, useMemo, useRef, useState } from "react";
import ReactFlow, {
  addEdge,
  Background,
  Connection,
  Controls,
  Edge,
  MiniMap,
  Node,
  ReactFlowInstance,
  useEdgesState,
  useNodesState,
} from "reactflow";
import "reactflow/dist/style.css";
import "../workspace.css";

import Sidebar from "../components/layout/Sidebar";
import Inspector from "../components/layout/Inspector";
import WorkflowReports from "../components/layout/WorkflowReports";
import { workflowView } from "../workflowView";
import fanucM710Graph from "../templates/fanuc-m710.json";
import fanucPreflightGraph from "../templates/fanuc-m710-preflight.json";
import fanucSnapshotGraph from "../templates/fanuc-m710-snapshot.json";
import fanucObserveGraph from "../templates/fanuc-m710-observe.json";
import { nodeTypes } from "../components/nodes";
import { bundleGraph, bundleView, executionGraph, graphId, graphList, instantiateGraph, parseWorkspace, serializeWorkspace, workspaceId } from "../graphWorkspace";
import {
  cancelGraphRuntime,
  getGraphRuntime,
  getRealSenseBridgeStatus,
  initIsaacGr00tScene,
  startRealSenseBridge,
  startGraphRuntime,
  stopRealSenseBridge,
} from "../api/graph";
import { API_BASE, uploadFile } from "../api/client";

import {
  canConnect,
  createNodeData,
  edgeKindFromPins,
  getSensorSourceMode,
  pinTypeOfNodeHandle,
} from "../nodeDefs";

const AUDIO_MIME_CANDIDATES = [
  "audio/webm;codecs=opus",
  "audio/webm",
  "audio/ogg;codecs=opus",
  "audio/ogg",
  "audio/mp4",
];

const ACTIVE_RUNTIME_STATES = new Set([
  "starting",
  "running",
  "waiting_feedback",
  "cancelling",
]);

const FINAL_RUNTIME_STATES = new Set(["completed", "cancelled", "error"]);
const RUNTIME_POLL_INTERVAL_MS = 1000;
const RGBD_RUNTIME_POLL_INTERVAL_MS = 250;
const RGBD_PREVIEW_POLL_INTERVAL_MS = 500;

function pickSupportedAudioMimeType() {
  if (typeof MediaRecorder === "undefined") return "";

  for (const candidate of AUDIO_MIME_CANDIDATES) {
    if (MediaRecorder.isTypeSupported(candidate)) {
      return candidate;
    }
  }

  return "";
}

function fileExtensionFromMimeType(mimeType: string) {
  if (mimeType.includes("webm")) return "webm";
  if (mimeType.includes("ogg")) return "ogg";
  if (mimeType.includes("mp4")) return "mp4";
  return "webm";
}

function summarizeLogValue(value: unknown, depth = 0): unknown {
  if (typeof value === "string") {
    if (value.startsWith("data:")) {
      return `[data-uri ${value.length} chars]`;
    }

    const compact = value.replace(/\s+/g, " ").trim();
    if (compact.length > 160) {
      return `${compact.slice(0, 157)}...`;
    }
    return compact;
  }

  if (value == null || typeof value !== "object") {
    return value;
  }

  if (depth >= 2) {
    if (Array.isArray(value)) {
      return `[${value.length} items]`;
    }
    return "{...}";
  }

  if (Array.isArray(value)) {
    const items = value.slice(0, 4).map((item) => summarizeLogValue(item, depth + 1));
    if (value.length > 4) {
      items.push(`... ${value.length - 4} more items`);
    }
    return items;
  }

  const entries = Object.entries(value as Record<string, unknown>);
  const summary: Record<string, unknown> = {};
  entries.slice(0, 6).forEach(([key, item]) => {
    summary[key] = summarizeLogValue(item, depth + 1);
  });
  if (entries.length > 6) {
    summary["..."] = `${entries.length - 6} more fields`;
  }
  return summary;
}

function summarizeForNodePreview(value: unknown, depth = 0): unknown {
  if (typeof value === "string") {
    const compact = value.replace(/\s+/g, " ").trim();
    if (compact.length > 260) {
      return `${compact.slice(0, 257)}...`;
    }
    return compact;
  }

  if (value == null || typeof value !== "object") {
    return value;
  }

  if (depth >= 3) {
    if (Array.isArray(value)) {
      return `[${value.length} items]`;
    }
    return "{...}";
  }

  if (Array.isArray(value)) {
    const items = value.slice(0, 8).map((item) => summarizeForNodePreview(item, depth + 1));
    if (value.length > 8) {
      items.push(`... ${value.length - 8} more items`);
    }
    return items;
  }

  const entries = Object.entries(value as Record<string, unknown>);
  const summary: Record<string, unknown> = {};
  entries.slice(0, 12).forEach(([key, item]) => {
    summary[key] = summarizeForNodePreview(item, depth + 1);
  });
  if (entries.length > 12) {
    summary["..."] = `${entries.length - 12} more fields`;
  }
  return summary;
}

function stringifyPreviewValue(value: unknown, max = 180) {
  if (value == null) return "";

  const raw =
    typeof value === "string"
      ? value
      : (() => {
          try {
            return JSON.stringify(value);
          } catch {
            return String(value);
          }
        })();

  const compact = String(raw).replace(/\s+/g, " ").trim();
  if (compact.length > max) {
    return `${compact.slice(0, max - 3)}...`;
  }
  return compact;
}


function extractImageFrameFromPayload(payload: any): string | undefined {
  const candidates = [payload?.image, payload?.data_url, payload?.frame, payload?.snapshot];
  for (const candidate of candidates) {
    if (typeof candidate === "string" && candidate.trim()) {
      return candidate.trim();
    }
  }
  return undefined;
}

function isUnityImagePreviewUrl(rawUrl: string) {
  const value = String(rawUrl ?? "").trim();
  if (!value) return false;

  try {
    const resolved = new URL(value, API_BASE);
    return resolved.pathname === "/api/unity/image";
  } catch {
    return false;
  }
}

function toFiniteNumber(value: unknown): number | null {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function buildLidarPointPreview(payload: any) {
  const lidar = payload?.lidar;
  if (!lidar || typeof lidar !== "object") {
    return undefined;
  }

  const rawPoints = Array.isArray(lidar.points) ? lidar.points : [];
  const points: number[][] = [];
  for (const rawPoint of rawPoints.slice(0, 600)) {
    if (!Array.isArray(rawPoint) || rawPoint.length < 2) {
      continue;
    }

    const x = toFiniteNumber(rawPoint[0]);
    const y = toFiniteNumber(rawPoint[1]);
    const z = toFiniteNumber(rawPoint[2]) ?? 0;
    if (x == null || y == null) {
      continue;
    }
    points.push([x, y, z]);
  }

  const summary = lidar.summary && typeof lidar.summary === "object" ? lidar.summary : {};
  const scan = lidar.scan && typeof lidar.scan === "object" ? lidar.scan : {};

  return {
    format: String(lidar.format ?? ""),
    frameId: String(lidar.frame_id ?? ""),
    pointCount: Number(lidar.point_count ?? points.length) || points.length,
    sampledCount: points.length,
    points,
    nearestRange:
      toFiniteNumber(summary.nearest_range) ??
      toFiniteNumber(summary.nearest_distance) ??
      null,
    obstacleCount: Number(summary.obstacle_count ?? 0) || 0,
    rangeMax: toFiniteNumber(scan.range_max) ?? toFiniteNumber(lidar.range_max) ?? null,
    sectors: summary.sectors && typeof summary.sectors === "object" ? summary.sectors : undefined,
  };
}

function buildSensorRuntimePreviewPatch(node: Node, payload: any) {
  const sourceMode = getSensorSourceMode(node.type, node.data?.sourceMode);
  const hasPayload = payload != null;
  const imageFrame =
    sourceMode === "remote_image" || sourceMode === "fused"
      ? extractImageFrameFromPayload(payload)
      : undefined;

  if (sourceMode !== "remote_lidar" && sourceMode !== "fused" && sourceMode !== "remote_image") {
    return {
      lastImageFrame: undefined,
      lastRawLidarJson: undefined,
      lastLidarOutputJson: undefined,
      lastLidarPointPreview: undefined,
    };
  }

  if (!hasPayload) {
    return {};
  }

  const patch: Record<string, unknown> = {};

  if (imageFrame) {
    patch.lastImageFrame = imageFrame;
  }

  if (sourceMode === "remote_lidar" || sourceMode === "fused") {
    patch.lastRawLidarJson =
      payload?.raw_lidar == null ? undefined : summarizeForNodePreview(payload.raw_lidar);
    patch.lastLidarOutputJson =
      payload?.lidar == null ? undefined : summarizeForNodePreview(payload.lidar);
    patch.lastLidarPointPreview = buildLidarPointPreview(payload);
  }

  return patch;
}

function isHttpUrl(rawUrl: string) {
  const value = String(rawUrl ?? "").trim();
  if (!value) return false;

  try {
    const resolved = new URL(value, API_BASE);
    return resolved.protocol === "http:" || resolved.protocol === "https:";
  } catch {
    return false;
  }
}

function browserPreviewUrl(rawUrl: string) {
  const value = String(rawUrl ?? "").trim();
  if (!value) return "";

  try {
    const resolved = new URL(value, API_BASE);
    if (resolved.hostname === "host.docker.internal") {
      resolved.hostname = "127.0.0.1";
    }
    return resolved.toString();
  } catch {
    return value;
  }
}

function realsensePortFromUrl(rawUrl: string) {
  try {
    return Number(new URL(String(rawUrl ?? "").trim(), API_BASE).port || 9180) || 9180;
  } catch {
    return 9180;
  }
}

function buildRgbdVisionRuntimePreviewPatch(node: Node, payload: any) {
  if (
    node.type !== "realsense_d435i_sensor" &&
    node.type !== "mediapipe_pose" &&
    node.type !== "rgbd_pose_fusion"
  ) {
    return {};
  }

  if (payload == null) {
    return {
      lastImageFrame: undefined,
      lastDepthImageFrame: undefined,
      lastVisionJson: undefined,
    };
  }

  const patch: Record<string, unknown> = {};
  const imageFrame = extractImageFrameFromPayload(payload);
  if (imageFrame) {
    patch.lastImageFrame = imageFrame;
  }

  if (typeof payload?.depth_image === "string" && payload.depth_image.trim()) {
    patch.lastDepthImageFrame = payload.depth_image.trim();
  }

  const previewPayload =
    payload?.sensor ??
    payload?.pose ??
    payload?.pose3d ??
    payload?.skeleton ??
    payload?.summary ??
    payload;
  patch.lastVisionJson = summarizeForNodePreview(previewPayload);
  return patch;
}

function buildRgbdPreviewPatchFromPayload(payload: any) {
  if (!payload || typeof payload !== "object") {
    return {};
  }

  const patch: Record<string, unknown> = {};
  const imageFrame = extractImageFrameFromPayload(payload);
  if (imageFrame) {
    patch.lastImageFrame = imageFrame;
  }

  const depthImage =
    typeof payload?.depth_image === "string"
      ? payload.depth_image
      : typeof payload?.depth?.preview_image === "string"
      ? payload.depth.preview_image
      : "";
  if (depthImage.trim()) {
    patch.lastDepthImageFrame = depthImage.trim();
  }

  const previewPayload = payload?.sensor ?? payload?.depth ?? payload;
  patch.lastVisionJson = summarizeForNodePreview(previewPayload);
  return patch;
}

function hasRgbdVisionNodes(nodes: Node[]) {
  return nodes.some(
    (node) =>
      node.type === "realsense_d435i_sensor" ||
      node.type === "mediapipe_pose" ||
      node.type === "rgbd_pose_fusion"
  );
}

function hasMotionLearningNodes(nodes: Node[]) {
  return nodes.some(
    (node) =>
      node.type === "video_motion_input" ||
      node.type === "motion_extract"
  );
}

function runtimePollIntervalMs(nodes: Node[]) {
  return hasRgbdVisionNodes(nodes) || hasMotionLearningNodes(nodes)
    ? RGBD_RUNTIME_POLL_INTERVAL_MS
    : RUNTIME_POLL_INTERVAL_MS;
}

function runtimeOptionsForGraph(nodes: Node[]) {
  if (!hasRgbdVisionNodes(nodes) && !hasMotionLearningNodes(nodes)) {
    return {};
  }

  return {
    persistent: true,
    require_feedback: false,
    continue_on_error: true,
    live_single_context: true,
    max_iterations: 0,
    cycle_interval_sec: 0.1,
    log_limit: 400,
  };
}

function extractPosePreviewFromPayload(payload: any) {
  const preview = payload?.pose_preview ?? payload?.posePreview;
  if (!preview || typeof preview !== "object") {
    return undefined;
  }

  const frames = (preview as Record<string, unknown>).frames;
  if (!Array.isArray(frames) || frames.length === 0) {
    return undefined;
  }

  return preview;
}

function buildVideoMotionRuntimePreviewPatch(
  node: Node,
  context: Record<string, any>,
  graphNodes: Node[],
  graphEdges: Edge[]
) {
  if (node.type !== "video_motion_input") {
    return {};
  }

  const motionExtractNodeIds = new Set(
    graphNodes
      .filter((candidate) => candidate.type === "motion_extract")
      .map((candidate) => candidate.id)
  );

  for (const edge of graphEdges) {
    if (edge.source !== node.id || !motionExtractNodeIds.has(edge.target)) {
      continue;
    }

    const preview = extractPosePreviewFromPayload(context[edge.target]);
    if (preview) {
      return { lastPosePreview: preview };
    }
  }

  return {};
}

function buildMotionExtractRuntimePatch(node: Node, payload: any) {
  if (node.type !== "motion_extract" || !payload || typeof payload !== "object") {
    return {};
  }

  const tracePath = String(payload.trace_path_host ?? payload.tracePathHost ?? payload.trace_path ?? payload.tracePath ?? "").trim();
  const trainCommand = String(payload.train_command ?? payload.trainCommand ?? payload.isaaclab_bridge?.powershell ?? "").trim();
  const sampledFrames = Number(payload.sampled_frames ?? payload.sampledFrames ?? 0);
  const trainingDataset = payload.training_dataset ?? payload.trainingDataset ?? {};
  const datasetPath = String(payload.dataset_path ?? trainingDataset.path ?? "").trim();
  const trainingSamples = Number(trainingDataset.samples ?? 0);
  const humanoidMeaning = payload.humanoid_meaning ?? payload.summary?.meaning_ko;
  const patch: Record<string, unknown> = {};

  if (tracePath) {
    patch.lastTracePath = tracePath;
  }
  if (trainCommand) {
    patch.lastIsaacLabTrainCommand = trainCommand;
  }
  if (Number.isFinite(sampledFrames) && sampledFrames > 0) {
    patch.lastMotionFrameCount = sampledFrames;
  }
  if (datasetPath) {
    patch.lastTrainingDatasetPath = datasetPath;
  }
  if (Number.isFinite(trainingSamples) && trainingSamples >= 0) {
    patch.lastTrainingSampleCount = trainingSamples;
  }
  if (Array.isArray(humanoidMeaning)) {
    patch.lastHumanoidMeaning = humanoidMeaning;
  }

  return patch;
}

function buildFanucIsaacLabRuntimePatch(node: Node, payload: any) {
  if (
    node.type !== "fanuc_omy3m_isaaclab_export" ||
    !payload ||
    typeof payload !== "object"
  ) {
    return {};
  }
  const dataset = payload.dataset ?? {};
  const tracePath = String(
    payload.trace_path_host ?? dataset.host_path ?? dataset.path ?? ""
  ).trim();
  const trainCommand = String(payload.train_command ?? "").trim();
  const frames = Number(payload.frame_count ?? payload.summary?.steps ?? 0);
  const patch: Record<string, unknown> = {};
  if (tracePath) patch.lastTracePath = tracePath;
  if (trainCommand) patch.lastIsaacLabTrainCommand = trainCommand;
  if (Number.isFinite(frames) && frames > 0) patch.lastMotionFrameCount = frames;
  return patch;
}

type RunTone = "idle" | "running" | "success" | "warning" | "error";

type StoryEntry = {
  id: string;
  state: RunTone;
  title: string;
  detail: string;
  summary: string;
  nodeId?: string;
  nodeType?: string;
  nodeLabel?: string;
};

type RunMetric = {
  label: string;
  value: string;
  detail: string;
  tone?: RunTone;
};

type RunOverview = {
  state: RunTone;
  title: string;
  detail: string;
  metrics: RunMetric[];
};

const NODE_LABEL_BY_TYPE: Record<string, string> = {
  prompt: "Prompt",
  image_input: "Image Input",
  sensor_input: "Sensor Input",
  sensor_camera: "Camera Sensor",
  sensor_remote_image: "Remote Image Sensor",
  sensor_remote_lidar: "Remote LiDAR Sensor",
  sensor_fused: "Fused Sensor",
  realsense_d435i_sensor: "RealSense D435i Sensor",
  mediapipe_pose: "MediaPipe Pose",
  rgbd_pose_fusion: "RGB-D Pose Fusion",
  video_motion_input: "Video Motion Input",
  motion_extract: "Motion Extract",
  motion_retarget: "Motion Retarget",
  robot_state: "Robot State",
  unity_state: "Robot State (Isaac Sim)",
  openvla_infer: "OpenVLA Infer",
  gr00t_infer: "NVIDIA GR00T Infer",
  action_decode: "Action Decode",
  g1_skill_decode: "G1 Skill Decode",
  robot_setup: "로봇 설정",
  safety_clamp: "Safety Clamp",
  robot_execute: "Robot Execute",
  dataset_upload: "Dataset Upload",
  dataset_preview: "Dataset Preview",
  openvla_finetune: "OpenVLA FineTune",
  speech_to_text: "Speech To Text",
  print: "Print",
};

const IDLE_OVERVIEW: RunOverview = {
  state: "idle",
  title: "그래프 준비 완료",
  detail: "실행하면 단계별 결과와 쉬운 설명이 여기에 정리됩니다.",
  metrics: [
    {
      label: "상태",
      value: "대기 중",
      detail: "아직 실행 전입니다.",
      tone: "idle",
    },
    {
      label: "실행 경로",
      value: "편집 중",
      detail: "현재 연결된 노드 순서가 표시됩니다.",
      tone: "idle",
    },
    {
      label: "최종 단계",
      value: "대기 중",
      detail: "마지막 노드 결과가 여기에 보입니다.",
      tone: "idle",
    },
    {
      label: "실행 대상",
      value: "미정",
      detail: "Robot Execute 대상과 모드가 정리됩니다.",
      tone: "idle",
    },
  ],
};

const IDLE_STORY: StoryEntry[] = [
  {
    id: "idle",
    state: "idle",
    title: "그래프가 실행 대기 중입니다.",
    detail: "Run Graph를 누르면 각 노드가 어떤 역할을 했는지 쉬운 문장으로 기록됩니다.",
    summary: "실행 전",
  },
];

function humanizeNodeType(type: string | undefined) {
  return NODE_LABEL_BY_TYPE[type ?? ""] ?? type ?? "Node";
}

function toneLabel(tone: RunTone) {
  if (tone === "running") return "실행 중";
  if (tone === "success") return "완료";
  if (tone === "warning") return "확인";
  if (tone === "error") return "오류";
  return "대기";
}

function clipText(value: unknown, max = 72) {
  const text = String(value ?? "").replace(/\s+/g, " ").trim();
  if (!text) return "";
  if (text.length <= max) return text;
  return `${text.slice(0, max - 1)}...`;
}

function speechTaskLabel(task: unknown) {
  return String(task ?? "transcribe") === "translate" ? "영문 번역" : "원문 추출";
}

function speechLanguageLabel(language: unknown) {
  const normalized = String(language ?? "auto");
  if (normalized === "ko") return "한국어";
  if (normalized === "en") return "영어";
  if (normalized === "ja") return "일본어";
  return "자동 감지";
}

function formatVec3(values: unknown) {
  if (!Array.isArray(values) || values.length === 0) return "0, 0, 0";
  return values
    .slice(0, 3)
    .map((value) => {
      const num = Number(value);
      return Number.isFinite(num) ? num.toFixed(2) : "0.00";
    })
    .join(", ");
}

function collectNumericTriples(value: unknown, out: number[][] = []) {
  if (!Array.isArray(value)) return out;

  const firstThree = value.slice(0, 3).map((item) => Number(item));
  if (value.length >= 3 && firstThree.every((item) => Number.isFinite(item))) {
    out.push(firstThree);
    return out;
  }

  value.forEach((item) => collectNumericTriples(item, out));
  return out;
}

function formatSmallNumber(value: number) {
  const normalized = Math.abs(value) < 0.0005 ? 0 : value;
  return normalized.toFixed(3);
}

function signedMaxComponent(triples: number[][], index: number) {
  let best = 0;
  triples.forEach((triple) => {
    const value = Number(triple[index]);
    if (Number.isFinite(value) && Math.abs(value) > Math.abs(best)) {
      best = value;
    }
  });
  return best;
}

function formatGr00tMovement(action: any) {
  const nav = action?.navigate_command ?? action?.["action.navigate_command"];
  const triples = collectNumericTriples(nav);
  if (triples.length > 0) {
    const forward = signedMaxComponent(triples, 0);
    const lateral = signedMaxComponent(triples, 1);
    const yaw = signedMaxComponent(triples, 2);
    return `전진 ${formatSmallNumber(forward)}, 좌우 ${formatSmallNumber(lateral)}, 회전 ${formatSmallNumber(yaw)}`;
  }

  if (action?.move_z != null || action?.move_x != null || action?.turn_yaw != null) {
    return `전진 ${formatSmallNumber(Number(action?.move_z ?? 0))}, 좌우 ${formatSmallNumber(
      Number(action?.move_x ?? 0)
    )}, 회전 ${formatSmallNumber(Number(action?.turn_yaw ?? 0))}`;
  }

  return "";
}

function formatPose(values: unknown) {
  if (!Array.isArray(values) || values.length === 0) return "";
  return values
    .slice(0, 6)
    .map((value) => {
      const num = Number(value);
      return Number.isFinite(num) ? num.toFixed(2) : "0.00";
    })
    .join(", ");
}

function formatGripperState(value: unknown) {
  if (typeof value === "boolean") {
    return value ? "open" : "close";
  }

  const text = String(value ?? "").trim();
  return text || "";
}

function primaryMessageFromPayload(payload: any): string {
  const candidates = [
    payload?.error,
    payload?.state?.message,
    payload?.state?.error,
    payload?.raw_state?.message,
    payload?.raw_state?.error,
    payload?.result?.error,
    payload?.result?.response?.error,
    payload?.response?.error,
    payload?.printed?.error,
    payload?.message,
    payload?.result?.message,
    payload?.result?.response?.message,
    payload?.response?.message,
    payload?.printed?.message,
  ];

  const found = candidates.find((value) => typeof value === "string" && value.trim());
  return found ? clipText(found) : "";
}

function buildRobotStateRuntimePreviewPatch(node: Node, payload: any) {
  if (node.type !== "robot_state" && node.type !== "unity_state") {
    return {
      lastStateMessage: undefined,
      lastStateScene: undefined,
      lastStateObject: undefined,
      lastStateSource: undefined,
      lastStateGripper: undefined,
      lastStatePose: undefined,
      lastStatePreview: undefined,
    };
  }

  const configuredState =
    node.type === "robot_state" && node.data?.robotState && typeof node.data.robotState === "object"
      ? node.data.robotState
      : undefined;
  const runtimeState =
    payload?.state && typeof payload.state === "object"
      ? payload.state
      : payload?.raw_state && typeof payload.raw_state === "object"
      ? payload.raw_state
      : configuredState;
  const stateRecord =
    runtimeState && typeof runtimeState === "object"
      ? (runtimeState as Record<string, unknown>)
      : undefined;

  const scene = stateRecord?.scene;
  const objectName = stateRecord?.object_name ?? stateRecord?.target;
  const source = stateRecord?.source ?? payload?.client_id;
  const gripper = stateRecord?.gripper_open ?? stateRecord?.gripper;
  const eefPose = stateRecord?.eef_pose;
  const pose =
    formatPose(eefPose) ||
    (Array.isArray(stateRecord?.position) || Array.isArray(stateRecord?.rotation)
      ? `p ${formatVec3(stateRecord?.position)} / r ${formatVec3(stateRecord?.rotation)}`
      : "");
  const previewSource = payload?.raw_state ?? payload?.state ?? configuredState ?? payload?.raw ?? null;

  return {
    lastStateMessage: primaryMessageFromPayload(payload) || undefined,
    lastStateScene: typeof scene === "string" && scene.trim() ? scene.trim() : undefined,
    lastStateObject:
      typeof objectName === "string" && objectName.trim() ? objectName.trim() : undefined,
    lastStateSource:
      typeof source === "string" && source.trim()
        ? source.trim()
        : node.type === "unity_state"
        ? String(node.data?.clientId ?? "default")
        : undefined,
    lastStateGripper: formatGripperState(gripper) || undefined,
    lastStatePose: pose || undefined,
    lastStatePreview:
      previewSource == null
        ? undefined
        : stringifyPreviewValue(summarizeForNodePreview(previewSource), 180),
  };
}

function payloadState(payload: any): RunTone {
  if (!payload || typeof payload !== "object") {
    return "success";
  }

  if (payload.ok === false || payload.error) {
    return "error";
  }

  if (payload.result?.ok === false || payload.result?.error) {
    return "error";
  }

  if (payload.response?.ok === false || payload.response?.error) {
    return "error";
  }

  if (payload.printed?.ok === false || payload.printed?.error) {
    return "error";
  }

  return "success";
}

function describeNodeExecution(node: any, payload: any): StoryEntry {
  const nodeType = String(node?.type ?? "");
  const nodeId = String(node?.id ?? "");
  const nodeLabel = String(node?.data?.label ?? humanizeNodeType(nodeType));
  const state = payloadState(payload);
  const message = primaryMessageFromPayload(payload);

  if (nodeType === "prompt") {
    const text = clipText(node?.data?.text ?? payload?.text ?? "지시문이 비어 있습니다.", 96);
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 준비 완료`,
      detail: `지시문 "${text}" 를 다음 추론 단계로 전달했습니다.`,
      summary: text || "지시문 준비",
    };
  }

  if (nodeType === "image_input") {
    const image = clipText(payload?.image ?? node?.data?.imageUrl ?? "이미지 없음", 84);
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 입력 완료`,
      detail: `이미지 소스를 읽어 추론 입력에 포함했습니다.`,
      summary: image || "이미지 입력",
    };
  }

  if (nodeType === "robot_state") {
    const robotState = payload?.state ?? node?.data?.robotState ?? {};
    const gripper = formatGripperState(robotState?.gripper_open ?? robotState?.gripper);
    const pose = formatPose(robotState?.eef_pose);
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 전달 완료`,
      detail:
        state === "error"
          ? message || "로봇 상태를 읽는 중 문제가 있었습니다."
          : `그리퍼 ${gripper || "unknown"} 상태와 ${pose ? `EEF pose ${pose}` : "엔드이펙터 자세"}를 추론 입력으로 전달했습니다.`,
      summary: gripper ? `state / ${gripper}` : "로봇 상태 전달",
    };
  }

  if (nodeType === "unity_state") {
    const unityState = payload?.state ?? payload?.raw_state ?? {};
    const clientId = clipText(payload?.client_id ?? node?.data?.clientId ?? "default", 28);
    const scene = clipText(unityState?.scene ?? "", 24);
    const objectName = clipText(unityState?.object_name ?? unityState?.target ?? "", 24);
    const position = Array.isArray(unityState?.position) ? formatVec3(unityState.position) : "";
    const gripper = formatGripperState(unityState?.gripper_open ?? unityState?.gripper);

    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: state === "error" ? `${nodeLabel} 확인 필요` : `${nodeLabel} 수신 완료`,
      detail:
        state === "error"
          ? message || "Unity 상태를 아직 받지 못했습니다."
          : `${clientId} 상태를 읽었습니다.${scene ? ` scene ${scene}` : ""}${objectName ? ` / object ${objectName}` : ""}${position ? ` / pos ${position}` : ""}${gripper ? ` / gripper ${gripper}` : ""}`,
      summary:
        state === "error"
          ? "Unity state 없음"
          : `${objectName || clientId}${gripper ? ` / ${gripper}` : ""}`,
    };
  }

  if (nodeType === "openvla_infer" || nodeType === "gr00t_infer") {
    const action = payload?.action ?? {};
    const model = clipText(
      payload?.model ?? node?.data?.modelName ?? (nodeType === "gr00t_infer" ? "GR00T" : "openvla"),
      40
    );
    const gripper = clipText(action?.gripper ?? "noop", 20);
    const gr00tMovement = nodeType === "gr00t_infer" ? formatGr00tMovement(action) : "";
    const movement = gr00tMovement || formatVec3(action?.translation);
    const targetPosition = Array.isArray(action?.target_position)
      ? formatVec3(action.target_position)
      : "";
    const patrolTarget = Array.isArray(action?.patrol_target_position)
      ? formatVec3(action.patrol_target_position)
      : "";
    const patrolDirection = Array.isArray(action?.patrol_direction)
      ? formatVec3(action.patrol_direction)
      : "";
    const patrolMode = String(action?.patrol_mode ?? "");
    const actionCommand = String(action?.command ?? payload?.task_analysis?.unity_command ?? "");
    const isPatrolAction =
      Boolean(patrolMode) ||
      actionCommand === "apply_patrol_control" ||
      String(action?.type ?? "") === "unity_patrol_action";
    const isWaypointAction =
      Boolean(targetPosition) ||
      actionCommand === "apply_base_waypoint" ||
      String(action?.type ?? "") === "unity_waypoint_action";
    const commandFamily = String(payload?.task_analysis?.command_family ?? "");
    const hideGripper = commandFamily === "locomotion" || Boolean(action?.gripper_suppressed);
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: state === "error" ? `${nodeLabel} 예측 실패` : `${nodeLabel} 예측 완료`,
      detail:
        state === "error"
          ? message || "모델 추론 단계에서 응답을 받지 못했습니다."
          : isPatrolAction
          ? `${model} 모델 출력을 Unity patrol ${patrolMode || "control"} ${patrolTarget || patrolDirection || movement}로 변환했습니다.`
          : isWaypointAction
          ? `${model} 모델 출력을 Unity waypoint 목표 ${targetPosition || movement}로 변환했습니다.`
          : nodeType === "gr00t_infer" && gr00tMovement
          ? `${model} 모델이 navigate_command ${movement}를 제안했습니다.`
          : hideGripper
          ? `${model} 모델이 이동 ${movement} 동작을 제안했습니다.`
          : `${model} 모델이 이동 ${movement}, 그리퍼 ${gripper} 동작을 제안했습니다.`,
      summary:
        state === "error"
          ? "추론 실패"
          : isPatrolAction
          ? `${model} -> patrol`
          : isWaypointAction
          ? `${model} -> waypoint`
          : hideGripper
          ? `${model} -> move`
          : `${model} -> ${gripper}`,
    };
  }

  if (nodeType === "action_decode") {
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 변환 완료`,
      detail: "모델 출력을 로봇이 바로 사용할 수 있는 액션 형식으로 해석했습니다.",
      summary: clipText(payload?.decoded?.type ?? payload?.decoded?.decodeMode ?? "액션 변환"),
    };
  }

  if (nodeType === "safety_clamp") {
    const maxDelta = payload?.safe_action?.safety?.maxDelta ?? node?.data?.maxDelta ?? 0.05;
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 안전 점검 완료`,
      detail: `이동과 회전을 안전 한도 ${maxDelta} 안으로 정리했습니다.`,
      summary: `안전 한도 ${maxDelta}`,
    };
  }

  if (nodeType === "robot_execute") {
    const result = payload?.result ?? payload ?? {};
    const mode = clipText(result?.mode ?? node?.data?.mode ?? "simulate", 20);
    const targetSource =
      result?.prim_path ??
      result?.primPath ??
      result?.object ??
      (String(node?.data?.mode ?? "").toLowerCase() === "omniverse"
        ? node?.data?.omniversePrimPath
        : node?.data?.unityObject) ??
      node?.data?.simulateName ??
      "robot";
    const target = clipText(targetSource, 32);
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: state === "error" ? `${nodeLabel} 실행 확인 필요` : `${nodeLabel} 실행 완료`,
      detail:
        state === "error"
          ? message || "실행 단계에서 응답을 확인하지 못했습니다."
          : `${mode} 모드로 ${target} 에 명령을 보냈고 ${clipText(result?.message ?? "실행 완료", 72)} 상태를 받았습니다.`,
      summary: state === "error" ? "실행 확인 필요" : `${mode} 실행 완료`,
    };
  }

  if (nodeType === "print") {
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: `${nodeLabel} 기록 완료`,
      detail: "최종 결과를 디버그 출력과 로그 패널에 남겼습니다.",
      summary: "최종 결과 기록",
    };
  }

  if (nodeType === "speech_to_text") {
    const task = String(node?.data?.task ?? "transcribe");
    const text = clipText(payload?.text ?? "", 96);
    const detectedLanguage = speechLanguageLabel(payload?.language ?? node?.data?.language ?? "auto");
    const modeLabel = speechTaskLabel(task);
    const detailPrefix =
      task === "translate"
        ? "음성을 영어 텍스트로 번역했습니다."
        : "음성을 원문 기준 텍스트로 옮겼습니다.";
    const textDetail = text
      ? ` 결과: "${text}"`
      : task === "translate"
      ? " 이번 입력에서는 영문 결과가 비어 있습니다."
      : " 이번 입력에서는 추출된 문장이 비어 있습니다.";
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: state === "error" ? `${nodeLabel} ${modeLabel} 실패` : `${nodeLabel} ${modeLabel} 완료`,
      detail:
        state === "error"
          ? message || "음성 인식 단계에서 응답을 받지 못했습니다."
          : `${detailPrefix} 감지 언어: ${detectedLanguage}.${textDetail}`,
      summary: state === "error" ? `${modeLabel} 실패` : text || modeLabel,
    };
  }

  if (nodeType === "openvla_finetune") {
    return {
      id: nodeId,
      nodeId,
      nodeType,
      nodeLabel,
      state,
      title: state === "error" ? `${nodeLabel} 시작 실패` : `${nodeLabel} 작업 시작`,
      detail:
        state === "error"
          ? message || "학습 작업 시작에 실패했습니다."
          : `파인튜닝 작업을 큐에 등록했습니다. ${message || ""}`.trim(),
      summary: state === "error" ? "학습 시작 실패" : "학습 큐 등록",
    };
  }

  return {
    id: nodeId,
    nodeId,
    nodeType,
    nodeLabel,
    state,
    title: state === "error" ? `${nodeLabel} 확인 필요` : `${nodeLabel} 완료`,
    detail: state === "error" ? message || "단계 실행에 문제가 있었습니다." : message || "단계를 정상적으로 처리했습니다.",
    summary: state === "error" ? "확인 필요" : "처리 완료",
  };
}

function buildRunPresentation(nodes: Node[], result: any) {
  const order = Array.isArray(result?.order) ? result.order : nodes.map((node) => node.id);
  const context = result?.context ?? {};
  const entries = order
    .map((nodeId) => {
      const node = nodes.find((item) => item.id === nodeId);
      if (!node) return null;
      return describeNodeExecution(node, context[nodeId]);
    })
    .filter(Boolean) as StoryEntry[];

  const errorCount = entries.filter((entry) => entry.state === "error").length;
  const finalEntry = entries[entries.length - 1];
  const robotEntry = entries.find((entry) => entry.nodeType === "robot_execute");
  const inferEntry = entries.find(
    (entry) => entry.nodeType === "openvla_infer" || entry.nodeType === "gr00t_infer"
  );
  const overviewState: RunTone = errorCount > 0 ? "warning" : "success";

  const overview: RunOverview = {
    state: overviewState,
    title: overviewState === "success" ? "그래프 실행 완료" : "실행 결과를 확인해 주세요",
    detail:
      overviewState === "success"
        ? `${entries.length}개 단계가 순서대로 완료되었습니다.`
        : `${errorCount}개 단계에서 추가 확인이 필요합니다.`,
    metrics: [
      {
        label: "상태",
        value: toneLabel(overviewState),
        detail: `${entries.length - errorCount}/${entries.length} 단계 정상 처리`,
        tone: overviewState,
      },
      {
        label: "실행 경로",
        value: `${entries.length} 단계`,
        detail: entries.map((entry) => entry.nodeLabel ?? humanizeNodeType(entry.nodeType)).join(" -> "),
        tone: "idle",
      },
      {
        label: "최종 단계",
        value: finalEntry?.title ?? "실행 대기",
        detail: finalEntry?.detail ?? "아직 실행 결과가 없습니다.",
        tone: finalEntry?.state ?? "idle",
      },
      {
        label: "실행 대상",
        value: robotEntry?.summary ?? "대기 중",
        detail: robotEntry?.detail ?? inferEntry?.detail ?? "로봇 실행 대상이 여기에 표시됩니다.",
        tone: robotEntry?.state ?? "idle",
      },
    ],
  };

  return {
    overview,
    story: entries,
    rawLogs: Array.isArray(result?.logs) ? result.logs : [],
  };
}

function runtimeTone(status: string): RunTone {
  if (status === "completed") return "success";
  if (status === "cancelled") return "warning";
  if (status === "error") return "error";
  if (ACTIVE_RUNTIME_STATES.has(status)) return "running";
  return "idle";
}

function runtimeStatusLabel(status: string) {
  if (status === "starting") return "시작 중";
  if (status === "running") return "반복 실행 중";
  if (status === "waiting_feedback") return "피드백 대기";
  if (status === "cancelling") return "종료 중";
  if (status === "completed") return "완료";
  if (status === "cancelled") return "취소됨";
  if (status === "error") return "오류";
  return "대기";
}

function buildRuntimePresentation(nodes: Node[], snapshot: any) {
  const status = String(snapshot?.status ?? "idle");
  const tone = runtimeTone(status);
  const lastResult = snapshot?.last_result ?? null;
  const base = lastResult
    ? buildRunPresentation(nodes, lastResult)
    : { overview: IDLE_OVERVIEW, story: [] as StoryEntry[], rawLogs: [] as string[] };
  const iteration = Number(snapshot?.iteration_count ?? 0);
  const frameCount = Number(snapshot?.frame_count ?? snapshot?.last_result?.frame_count ?? 0);
  const activeNodeType = clipText(
    snapshot?.active_node_type ? humanizeNodeType(String(snapshot.active_node_type)) : "",
    48
  );
  const activeNodeId = String(snapshot?.active_node_id ?? "");
  const sessionMessage = clipText(snapshot?.error ?? snapshot?.message ?? "", 160);
  const loopEnabled = Boolean(snapshot?.options?.persistent);
  const liveSingleContext = Boolean(snapshot?.options?.live_single_context);
  const pathDetail =
    nodes.length > 0
      ? nodes.map((node) => String(node.data?.label ?? humanizeNodeType(node.type))).join(" -> ")
      : "그래프 없음";
  const latestStageEntry =
    base.story.find((entry) => entry.nodeId === activeNodeId) ?? base.story[base.story.length - 1];

  let title = "그래프 준비 완료";
  let detail = "실행 대기 중입니다.";
  let currentStageValue = latestStageEntry?.title ?? "실행 대기";
  let currentStageDetail = latestStageEntry?.detail ?? "아직 실행 결과가 없습니다.";
  let loopValue = liveSingleContext
    ? frameCount > 0
      ? `1회차 / ${frameCount}프레임`
      : "1회차 초기화 중"
    : loopEnabled
    ? iteration > 0
      ? `${iteration}회차`
      : "초기화 중"
    : "단발 실행";
  let loopDetail = loopEnabled
    ? liveSingleContext
      ? "한 번 올라간 라이브 세션 안에서 새 프레임만 계속 처리합니다."
      : "메모리에 올라간 런타임이 외부 피드백을 받아 다음 명령을 준비합니다."
    : "피드백 루프 없이 1회 실행 후 종료됩니다.";

  if (status === "starting") {
    title = "그래프 런타임 시작 중";
    detail = sessionMessage || "서버 메모리에 그래프 세션을 올리고 있습니다.";
    currentStageValue = "메모리 적재 중";
    currentStageDetail = "그래프 구조와 실행 순서를 준비하고 있습니다.";
  } else if (status === "running") {
    title = "그래프 런타임 실행 중";
    detail =
      sessionMessage ||
      (liveSingleContext
        ? `1회차 라이브 세션에서 ${frameCount > 0 ? `${frameCount}번째 프레임` : "첫 프레임"}을 처리 중입니다.`
        : `${iteration > 0 ? `${iteration}회차` : "첫 회차"} 그래프를 실행하며 외부 실행기와 왕복 중입니다.`);
    currentStageValue = activeNodeType || latestStageEntry?.title || "노드 실행 중";
    currentStageDetail =
      sessionMessage ||
      latestStageEntry?.detail ||
      "현재 노드의 결과를 만들고 다음 명령을 계산하고 있습니다.";
  } else if (status === "waiting_feedback") {
    title = "외부 처리 결과 대기 중";
    detail =
      sessionMessage ||
      `${iteration > 0 ? `${iteration}회차 명령을 보낸 뒤` : "시작 직후"} 최신 Unity 상태를 기다리고 있습니다.`;
    currentStageValue = "피드백 대기";
    currentStageDetail =
      sessionMessage || "Unity가 보낸 최신 상태를 받은 뒤 다음 회차 분석을 시작합니다.";
  } else if (status === "cancelling") {
    title = "그래프 런타임 종료 중";
    detail = sessionMessage || "현재 진행 중인 사이클을 마무리한 뒤 메모리에서 내립니다.";
    currentStageValue = "종료 처리 중";
    currentStageDetail = "취소 요청을 반영하고 있습니다.";
  } else if (status === "completed") {
    title = loopEnabled ? "그래프 런타임 완료" : base.overview.title;
    detail =
      sessionMessage ||
      (liveSingleContext
        ? `${frameCount}개 프레임 처리 후 라이브 세션이 종료되었습니다.`
        : loopEnabled
        ? `${iteration}회 반복 실행 후 종료되었습니다.`
        : base.overview.detail);
    currentStageValue = latestStageEntry?.title ?? "실행 완료";
    currentStageDetail = latestStageEntry?.detail ?? detail;
    loopDetail = loopEnabled
      ? "런타임이 정상 종료되어 그래프는 메모리에서 내려갔습니다."
      : "단발 실행이 정상 종료되었습니다.";
  } else if (status === "cancelled") {
    title = "그래프 런타임 취소됨";
    detail = sessionMessage || "취소 요청에 따라 그래프를 메모리에서 내렸습니다.";
    currentStageValue = "취소 완료";
    currentStageDetail = "새 실행을 시작하면 새 세션으로 다시 올라갑니다.";
    loopDetail = "현재 세션은 종료되었고 그래프는 메모리에 남아 있지 않습니다.";
  } else if (status === "error") {
    title = "그래프 런타임 오류";
    detail = sessionMessage || "런타임이 오류로 중지되었습니다.";
    currentStageValue = latestStageEntry?.title ?? "오류 발생";
    currentStageDetail = sessionMessage || latestStageEntry?.detail || "오류 원인을 확인해 주세요.";
    loopDetail = "오류로 인해 그래프 런타임이 메모리에서 내려갔습니다.";
  } else if (lastResult) {
    title = base.overview.title;
    detail = base.overview.detail;
    currentStageValue = base.overview.metrics[2]?.value ?? currentStageValue;
    currentStageDetail = base.overview.metrics[2]?.detail ?? currentStageDetail;
  }

  const runtimeEntry: StoryEntry = {
    id: `runtime-${status}-${iteration}`,
    state: tone,
    title,
    detail,
    summary: runtimeStatusLabel(status),
  };

  let story = base.story.length > 0 ? base.story : [];
  if (status === "starting" || status === "waiting_feedback" || status === "cancelling") {
    story = [runtimeEntry, ...story];
  } else if (status === "cancelled" || status === "error") {
    story = [runtimeEntry, ...story];
  } else if (story.length === 0) {
    story = [runtimeEntry];
  }

  const overview: RunOverview = {
    state: tone,
    title,
    detail,
    metrics: [
      {
        label: "상태",
        value: runtimeStatusLabel(status),
        detail:
          sessionMessage ||
          (liveSingleContext
            ? `1회차 라이브 세션에서 ${frameCount > 0 ? `${frameCount}프레임` : "프레임 초기화"} 기준으로 유지되고 있습니다.`
            : loopEnabled
            ? `${iteration > 0 ? `${iteration}회차` : "첫 회차"} 기준으로 런타임이 유지되고 있습니다.`
            : "이번 실행은 단발 실행으로 처리됩니다."),
        tone,
      },
      {
        label: "실행 경로",
        value: `${nodes.length} 단계`,
        detail: pathDetail,
        tone: "idle",
      },
      {
        label: "현재 단계",
        value: currentStageValue,
        detail: currentStageDetail,
        tone: tone === "warning" ? "warning" : ACTIVE_RUNTIME_STATES.has(status) ? "running" : tone,
      },
      {
        label: "반복 상태",
        value: loopValue,
        detail: loopDetail,
        tone: loopEnabled ? tone : "idle",
      },
    ],
  };

  return {
    overview,
    story,
    logs:
      Array.isArray(snapshot?.logs) && snapshot.logs.length > 0
        ? snapshot.logs
        : base.rawLogs.length > 0
        ? base.rawLogs
        : ["Runtime session ready"],
    context: lastResult?.context ?? {},
    sessionStatus: status,
  };
}

const initialNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 100 },
    data: {
      label: "Prompt Node",
      text: "walk forward toward the checkpoint and turn slightly left",
    },
  },
  {
    id: "node-2",
    type: "sensor_remote_image",
    position: { x: 80, y: 260 },
    data: {
      label: "Isaac Sim Camera Sensor",
      remoteImageUrl: "http://localhost:9000/api/unity/image?client_id=isaacsim01",
      remoteImageTransport: "snapshot",
      remoteMethod: "GET",
      remoteHeaders: {},
      remoteBody: {},
      requestTimeoutSec: 10,
      rtspTransport: "tcp",
      ffmpegPath: "ffmpeg",
    },
  },
  {
    id: "node-3",
    type: "unity_state",
    position: { x: 80, y: 420 },
    data: {
      label: "Robot State (Isaac Sim)",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },
  {
    id: "node-4",
    type: "openvla_infer",
    position: { x: 440, y: 250 },
    data: {
      label: "OpenVLA Infer Node",
      modelName: "openvla-stub",
    },
  },
  {
    id: "node-5",
    type: "action_decode",
    position: { x: 780, y: 250 },
    data: {
      label: "Action Decode Node",
      decodeMode: "default",
    },
  },
  {
    id: "node-6",
    type: "safety_clamp",
    position: { x: 1120, y: 250 },
    data: {
      label: "Safety Clamp Node",
      enabled: true,
      maxDelta: 0.05,
    },
  },
  {
    id: "node-7",
    type: "robot_execute",
    position: { x: 1460, y: 250 },
    data: {
      label: "Robot Execute Node",
      executor: "robot-executor",
      mode: "omniverse",
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GR00T_Humanoid",
      omniverseCommand: "execute_policy",
      omniverseTimeoutSec: 15,
    },
  },
  {
    id: "node-8",
    type: "print",
    position: { x: 1800, y: 250 },
    data: { label: "Print Node" },
  },
];

const initialEdges: Edge[] = [
  {
    id: "e1-4",
    source: "node-1",
    target: "node-4",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e2-4",
    source: "node-2",
    target: "node-4",
    sourceHandle: "image",
    targetHandle: "image",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e3-4",
    source: "node-3",
    target: "node-4",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "e4-5",
    source: "node-4",
    target: "node-5",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#c084fc", strokeWidth: 1.5 },
  },
  {
    id: "e5-6",
    source: "node-5",
    target: "node-6",
    sourceHandle: "decoded",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e6-7",
    source: "node-6",
    target: "node-7",
    sourceHandle: "safe_action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#f59e0b", strokeWidth: 1.5 },
  },
  {
    id: "e7-8",
    source: "node-7",
    target: "node-8",
    sourceHandle: "result",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

type GraphTemplateId =
  | "openvla"
  | "gr00t"
  | "omy3m"
  | "fanuc_m710"
  | "fanuc_m710_world_observe"
  | "fanuc_m710_snapshot"
  | "fanuc_m710_preflight"
  | "fanuc_omy3m"
  | "motion"
  | "rgbd_follow"
  | "spatial_collection"
  | "spatial_rag"
  | "model_sync";

type GraphTemplate = {
  id: GraphTemplateId;
  label: string;
  description: string;
  nodes: Node[];
  edges: Edge[];
};

const gr00tTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 100 },
    data: {
      label: "Prompt Node",
      text: "walk forward toward the checkpoint and turn slightly left",
    },
  },
  {
    id: "node-2",
    type: "sensor_remote_image",
    position: { x: 80, y: 260 },
    data: {
      label: "Isaac Sim Camera Sensor",
      remoteImageUrl: "http://localhost:9000/api/unity/image?client_id=isaacsim01",
      remoteImageTransport: "snapshot",
      remoteMethod: "GET",
      remoteHeaders: {},
      remoteBody: {},
      requestTimeoutSec: 10,
      rtspTransport: "tcp",
      ffmpegPath: "ffmpeg",
    },
  },
  {
    id: "node-3",
    type: "unity_state",
    position: { x: 80, y: 420 },
    data: {
      label: "Robot State (Isaac Sim)",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },
  {
    id: "node-4",
    type: "gr00t_infer",
    position: { x: 440, y: 250 },
    data: {
      label: "NVIDIA GR00T Infer Node",
      modelName: "nvidia/GR00T-N1.7-3B",
      endpoint: "",
      embodimentTag: "REAL_G1",
      inferenceMode: "pytorch",
      actionHorizon: 40,
      actionFormat: "humanoid_motion",
      unityCommand: "apply_humanoid_motion",
      waypointSpeed: 1.0,
      requireImage: true,
    },
  },
  {
    id: "node-5",
    type: "action_decode",
    position: { x: 780, y: 250 },
    data: {
      label: "Action Decode Node",
      decodeMode: "g1_skill",
    },
  },
  {
    id: "node-6",
    type: "safety_clamp",
    position: { x: 1120, y: 250 },
    data: {
      label: "Safety Clamp Node",
      enabled: true,
      maxDelta: 0.5,
    },
  },
  {
    id: "node-7",
    type: "robot_execute",
    position: { x: 1460, y: 250 },
    data: {
      label: "Robot Execute Node",
      executor: "robot-executor",
      mode: "omniverse",
      simulateName: "default-sim",
      simulateSpeed: 1.0,
      dryRun: false,
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GR00T_Humanoid",
      omniverseCommand: "apply_humanoid_motion",
      omniverseTimeoutSec: 15,
    },
  },
  {
    id: "node-8",
    type: "print",
    position: { x: 1800, y: 250 },
    data: { label: "Print Node" },
  },
];

const modelSyncTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "model_sync",
    position: { x: 120, y: 180 },
    data: {
      label: "Sync Cosmos Model",
      sourcePath: "/host_downloads/Cosmos-Reason2-2B",
      targetName: "Cosmos-Reason2-2B",
      overwrite: false,
      requiredFiles: "config.json",
    },
  },
  {
    id: "node-2",
    type: "print",
    position: { x: 520, y: 180 },
    data: {
      label: "Print Sync Result",
    },
  },
];

const modelSyncTemplateEdges: Edge[] = [
  {
    id: "e1-2",
    source: "node-1",
    target: "node-2",
    sourceHandle: "status",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const gr00tTemplateEdges: Edge[] = [
  {
    id: "e1-4",
    source: "node-1",
    target: "node-4",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e2-4",
    source: "node-2",
    target: "node-4",
    sourceHandle: "image",
    targetHandle: "image",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e3-4",
    source: "node-3",
    target: "node-4",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "e4-5",
    source: "node-4",
    target: "node-5",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#c084fc", strokeWidth: 1.5 },
  },
  {
    id: "e5-6",
    source: "node-5",
    target: "node-6",
    sourceHandle: "decoded",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e6-7",
    source: "node-6",
    target: "node-7",
    sourceHandle: "safe_action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#f59e0b", strokeWidth: 1.5 },
  },
  {
    id: "e7-8",
    source: "node-7",
    target: "node-8",
    sourceHandle: "result",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const motionTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 120 },
    data: {
      label: "Motion Prompt",
      text: "imitate the uploaded human motion",
    },
  },
  {
    id: "node-2",
    type: "video_motion_input",
    position: { x: 80, y: 300 },
    data: {
      label: "Human Motion Video",
      videoPath: "",
      motionLabel: "",
      prompt: "",
    },
  },
  {
    id: "node-3",
    type: "motion_extract",
    position: { x: 440, y: 220 },
    data: {
      label: "Motion Extract Node",
      everyNFrames: 1,
      maxFrames: 0,
      minVisibility: 0.15,
      includeFrames: false,
      outputName: "",
    },
  },
  {
    id: "node-4",
    type: "motion_retarget",
    position: { x: 800, y: 220 },
    data: {
      label: "Motion Retarget Node",
      speed: 0.45,
      stepScale: 0.25,
      turnScale: 0.2,
      durationSec: 0.0,
      fullBodyRetarget: true,
      targetEmbodiment: "REAL_G1",
      emitG1JointTargets: false,
      emitG1Trajectory: false,
      streamVideoFrames: false,
      streamJointScale: 0.62,
      streamRootLateralScale: 0.9,
      streamRootVerticalScale: 1.8,
      streamRootDepthScale: 0.25,
      streamRootMaxLateral: 0.12,
      streamRootMaxVertical: 0.18,
      streamRootMaxDepth: 0.08,
      useIsaacUsdIk: false,
      rootMotionEnabled: true,
      rootTranslationEnabled: true,
      rootMotionMode: "in_place",
      rootLateralScale: 0.0,
      rootVerticalScale: 1.15,
      rootDepthScale: 0.35,
      rootImageXDepthScale: 1.2,
      rootMirrorImageX: true,
      rootMaxLateral: 0.0,
      rootMaxVertical: 0.18,
      rootMaxDepth: 0.12,
      rootPitchEnabled: true,
      rootMaxPitchDeg: 10.0,
      jointSmoothing: 0.22,
      jointMaxDeltaRad: 0.10,
      jointAverageWindow: 1,
      jointSelection: "recent",
      maxPoseFrames: 120,
      poseScale: 1.0,
      lowerBodyScale: 1.15,
      poseCalibration: "none",
      lowerBodyMode: "full_body",
      inactiveArmDeadzone: 0.0,
      limbMinVisibility: 0.3,
    },
  },
  {
    id: "node-5",
    type: "robot_execute",
    position: { x: 1160, y: 220 },
    data: {
      label: "Apply To G1 Humanoid",
      executor: "robot-executor",
      mode: "omniverse",
      simulateName: "default-sim",
      simulateSpeed: 1.0,
      dryRun: false,
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GR00T_Humanoid",
      omniverseCommand: "apply_humanoid_motion",
      omniverseTimeoutSec: 15,
      omniverseWaitForResponse: false,
    },
  },
  {
    id: "node-6",
    type: "print",
    position: { x: 1520, y: 220 },
    data: { label: "Print Omniverse Result" },
  },
];

const motionTemplateEdges: Edge[] = [
  {
    id: "e1-3",
    source: "node-1",
    target: "node-3",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e2-3-video",
    source: "node-2",
    target: "node-3",
    sourceHandle: "video",
    targetHandle: "video",
    animated: true,
    style: { stroke: "#f97316", strokeWidth: 1.5 },
  },
  {
    id: "e2-3-label",
    source: "node-2",
    target: "node-3",
    sourceHandle: "label",
    targetHandle: "label",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e3-4",
    source: "node-3",
    target: "node-4",
    sourceHandle: "motion",
    targetHandle: "motion",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e3-4-skill",
    source: "node-3",
    target: "node-4",
    sourceHandle: "skill",
    targetHandle: "skill",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e1-4",
    source: "node-1",
    target: "node-4",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e4-5",
    source: "node-4",
    target: "node-5",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#c084fc", strokeWidth: 1.5 },
  },
  {
    id: "e5-6",
    source: "node-5",
    target: "node-6",
    sourceHandle: "result",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const rgbdFollowTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 80 },
    data: {
      label: "Follow Prompt",
      text: "follow the human pose from the RealSense D435i camera",
    },
  },
  {
    id: "node-2",
    type: "realsense_d435i_sensor",
    position: { x: 80, y: 250 },
    data: {
      label: "RealSense D435i Sensor",
      realsenseSerial: "",
      remoteRgbdUrl: "http://host.docker.internal:9180/frame",
      captureWidth: 640,
      captureHeight: 480,
      fps: 15,
      frameTimeoutMs: 5000,
      warmupFrames: 4,
      depthMinMeters: 0.2,
      depthMaxMeters: 4.0,
      mockWhenUnavailable: false,
    },
  },
  {
    id: "node-3",
    type: "mediapipe_pose",
    position: { x: 460, y: 200 },
    data: {
      label: "MediaPipe Pose",
      modelComplexity: 1,
      staticImageMode: false,
      minDetectionConfidence: 0.5,
      minTrackingConfidence: 0.5,
      minPoseVisibility: 0.55,
    },
  },
  {
    id: "node-4",
    type: "rgbd_pose_fusion",
    position: { x: 820, y: 220 },
    data: {
      label: "RGB-D Pose Fusion",
      depthSampleRadius: 3,
      minPoseVisibility: 0.55,
      landmarkSmoothing: 0.45,
      landmarkMaxDelta: 0.18,
      requireStableTorso: false,
      holdLargeJumps: true,
      maxMotionFrames: 1,
      motionLabel: "",
      prompt: "",
    },
  },
  {
    id: "node-5",
    type: "motion_retarget",
    position: { x: 1180, y: 220 },
    data: {
      label: "Humanoid Retarget",
      speed: 0.45,
      stepScale: 0.22,
      turnScale: 0.15,
      durationSec: 0.0,
      fullBodyRetarget: true,
      targetEmbodiment: "REAL_G1",
      emitG1JointTargets: true,
      useIsaacUsdIk: false,
      rootMotionEnabled: true,
      jointSmoothing: 0.35,
      jointMaxDeltaRad: 0.08,
      jointDeadbandRad: 0.025,
      jointAverageWindow: 1,
      jointSelection: "latest",
      maxPoseFrames: 1,
      poseScale: 1.05,
      poseCalibration: "none",
      lowerBodyMode: "full_body",
      lowerBodyScale: 0.75,
      inactiveArmDeadzone: 0.18,
      limbMinVisibility: 0.42,
    },
  },
  {
    id: "node-6",
    type: "robot_execute",
    position: { x: 1540, y: 220 },
    data: {
      label: "Apply To Isaac Humanoid",
      executor: "robot-executor",
      mode: "omniverse",
      simulateName: "default-sim",
      simulateSpeed: 1.0,
      dryRun: false,
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GR00T_Humanoid",
      omniverseCommand: "apply_humanoid_motion",
      omniverseTimeoutSec: 10,
      omniverseWaitForResponse: false,
    },
  },
  {
    id: "node-7",
    type: "print",
    position: { x: 1900, y: 220 },
    data: { label: "Print Isaac Result" },
  },
];

const rgbdFollowTemplateEdges: Edge[] = [
  {
    id: "e1-4",
    source: "node-1",
    target: "node-4",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e1-5",
    source: "node-1",
    target: "node-5",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e2-3-image",
    source: "node-2",
    target: "node-3",
    sourceHandle: "image",
    targetHandle: "image",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e2-4-depth",
    source: "node-2",
    target: "node-4",
    sourceHandle: "depth",
    targetHandle: "depth",
    animated: true,
    style: { stroke: "#8b5cf6", strokeWidth: 1.5 },
  },
  {
    id: "e3-4-pose",
    source: "node-3",
    target: "node-4",
    sourceHandle: "pose",
    targetHandle: "pose",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e3-4-image",
    source: "node-3",
    target: "node-4",
    sourceHandle: "image",
    targetHandle: "image",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e4-5-motion",
    source: "node-4",
    target: "node-5",
    sourceHandle: "motion",
    targetHandle: "motion",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e4-5-skill",
    source: "node-4",
    target: "node-5",
    sourceHandle: "skill",
    targetHandle: "skill",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e5-6",
    source: "node-5",
    target: "node-6",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#c084fc", strokeWidth: 1.5 },
  },
  {
    id: "e6-7",
    source: "node-6",
    target: "node-7",
    sourceHandle: "result",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const spatialCollectionTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 120 },
    data: {
      label: "Spatial Task Prompt",
      text: "Move to the front of the red box.",
    },
  },
  {
    id: "node-2",
    type: "unity_state",
    position: { x: 80, y: 300 },
    data: {
      label: "Robot State (Isaac Sim)",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },
  {
    id: "node-3",
    type: "scene_scan",
    position: { x: 440, y: 220 },
    data: {
      label: "Scene Scan",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniversePrimPath: "/World/GR00T_Humanoid",
      sceneRootPath: "/World",
      maxObjects: 256,
      timeoutSec: 15,
    },
  },
  {
    id: "node-4",
    type: "scene_context_reset",
    position: { x: 800, y: 220 },
    data: {
      label: "Scene Context Reset",
      clearSceneMemory: false,
      memoryPath: "spatial_memory/spatial_memory.jsonl",
    },
  },
  {
    id: "node-5",
    type: "spatial_memory_save",
    position: { x: 1160, y: 220 },
    data: {
      label: "Spatial Memory Save",
      memoryPath: "spatial_memory/spatial_memory.jsonl",
    },
  },
  {
    id: "node-6",
    type: "print",
    position: { x: 1520, y: 220 },
    data: { label: "Print Memory Record" },
  },
];

const spatialCollectionTemplateEdges: Edge[] = [
  {
    id: "e2-3-state",
    source: "node-2",
    target: "node-3",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "e3-4-scan",
    source: "node-3",
    target: "node-4",
    sourceHandle: "scan",
    targetHandle: "scan",
    animated: true,
    style: { stroke: "#8b5cf6", strokeWidth: 1.5 },
  },
  {
    id: "e4-5-scene",
    source: "node-4",
    target: "node-5",
    sourceHandle: "scene",
    targetHandle: "scan",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e1-5-prompt",
    source: "node-1",
    target: "node-5",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e5-6-record",
    source: "node-5",
    target: "node-6",
    sourceHandle: "record",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const spatialRagTemplateNodes: Node[] = [
  {
    id: "node-1",
    type: "prompt",
    position: { x: 80, y: 120 },
    data: {
      label: "Spatial Task Prompt",
      text: "Move to the front of the red box.",
    },
  },
  {
    id: "node-2",
    type: "unity_state",
    position: { x: 80, y: 300 },
    data: {
      label: "Robot State (Isaac Sim)",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },
  {
    id: "node-3",
    type: "scene_scan",
    position: { x: 420, y: 220 },
    data: {
      label: "Scene Scan",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniversePrimPath: "/World/GR00T_Humanoid",
      sceneRootPath: "/World",
      maxObjects: 256,
      timeoutSec: 15,
    },
  },
  {
    id: "node-4",
    type: "scene_context_reset",
    position: { x: 760, y: 220 },
    data: {
      label: "Scene Context Reset",
      clearSceneMemory: false,
      memoryPath: "spatial_memory/spatial_memory.jsonl",
    },
  },
  {
    id: "node-5",
    type: "spatial_rag_retrieve",
    position: { x: 1100, y: 220 },
    data: {
      label: "Spatial RAG Retrieve",
      memoryPath: "spatial_memory/spatial_memory.jsonl",
      topK: 5,
    },
  },
  {
    id: "node-6",
    type: "spatial_context_builder",
    position: { x: 1440, y: 220 },
    data: {
      label: "Spatial Context Builder",
      maxLines: 5,
    },
  },
  {
    id: "node-7",
    type: "g1_skill_decode",
    position: { x: 1780, y: 220 },
    data: {
      label: "G1 Skill Decode",
      defaultSpeed: 0.55,
    },
  },
  {
    id: "node-8",
    type: "safety_clamp",
    position: { x: 2120, y: 220 },
    data: {
      label: "Safety Clamp Node",
      enabled: true,
      maxDelta: 1.0,
    },
  },
  {
    id: "node-9",
    type: "robot_execute",
    position: { x: 2460, y: 220 },
    data: {
      label: "Robot Execute Node",
      executor: "robot-executor",
      mode: "omniverse",
      dryRun: false,
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GR00T_Humanoid",
      omniverseCommand: "apply_humanoid_motion",
      omniverseTimeoutSec: 15,
    },
  },
  {
    id: "node-10",
    type: "print",
    position: { x: 2800, y: 220 },
    data: { label: "Print Execute Result" },
  },
];

const spatialRagTemplateEdges: Edge[] = [
  {
    id: "e2-3-state",
    source: "node-2",
    target: "node-3",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "e3-4-scan",
    source: "node-3",
    target: "node-4",
    sourceHandle: "scan",
    targetHandle: "scan",
    animated: true,
    style: { stroke: "#8b5cf6", strokeWidth: 1.5 },
  },
  {
    id: "e1-5-prompt",
    source: "node-1",
    target: "node-5",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e4-5-scene",
    source: "node-4",
    target: "node-5",
    sourceHandle: "scene",
    targetHandle: "scan",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e4-5-scene-id",
    source: "node-4",
    target: "node-5",
    sourceHandle: "scene_id",
    targetHandle: "scene_id",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "e1-6-prompt",
    source: "node-1",
    target: "node-6",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e5-6-memories",
    source: "node-5",
    target: "node-6",
    sourceHandle: "memories",
    targetHandle: "memories",
    animated: true,
    style: { stroke: "#8b5cf6", strokeWidth: 1.5 },
  },
  {
    id: "e4-6-scene",
    source: "node-4",
    target: "node-6",
    sourceHandle: "scene",
    targetHandle: "scan",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e6-7-prompt",
    source: "node-6",
    target: "node-7",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "e2-7-state",
    source: "node-2",
    target: "node-7",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "e7-8-action",
    source: "node-7",
    target: "node-8",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#c084fc", strokeWidth: 1.5 },
  },
  {
    id: "e8-9-safe-action",
    source: "node-8",
    target: "node-9",
    sourceHandle: "safe_action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "e9-10-result",
    source: "node-9",
    target: "node-10",
    sourceHandle: "result",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const omy3mTemplateNodes: Node[] = [
  {
    id: "omy3m-prompt",
    type: "prompt",
    position: { x: 60, y: 80 },
    data: {
      label: "OMY3M 작업 지시",
      text: "빨간 큐브를 집어서 오른쪽 트레이로 옮겨라",
    },
  },
  {
    id: "omy3m-camera",
    type: "sensor_remote_image",
    position: { x: 60, y: 250 },
    data: {
      label: "OMY3M 손목 카메라",
      remoteImageUrl: "http://localhost:9000/api/unity/image?client_id=isaacsim01",
      remoteImageTransport: "snapshot",
      remoteMethod: "GET",
      remoteHeaders: {},
      remoteBody: {},
      requestTimeoutSec: 10,
      rtspTransport: "tcp",
      ffmpegPath: "ffmpeg",
    },
  },
  {
    id: "omy3m-state",
    type: "unity_state",
    position: { x: 60, y: 430 },
    data: {
      label: "OMY3M 로봇 상태",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },
  {
    id: "omy3m-action",
    type: "omy3m_joint_action",
    position: { x: 430, y: 150 },
    data: {
      label: "OMY3M 6축 관절 Action",
      jointNames: "joint1,joint2,joint3,joint4,joint5,joint6",
      jointPositions: "0,-0.5,0.8,0,0.4,0",
      gripperPosition: 0,
      jointLimitRad: 3.14159,
      durationSec: 1,
    },
  },
  {
    id: "omy3m-execute",
    type: "robot_execute",
    position: { x: 790, y: 150 },
    data: {
      label: "OMY3M Isaac Sim 실행",
      executor: "robot-executor",
      mode: "omniverse",
      dryRun: false,
      robotType: "articulated_arm",
      controllerMode: "joint",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/OMY3M",
      omniverseStage: "/World",
      omniversePrimPath: "/World/OMY3M_Learning/Robot",
      omniverseCommand: "apply_joint_targets",
      omniverseTimeoutSec: 15,
      omniverseWaitForResponse: true,
    },
  },
  {
    id: "omy3m-record",
    type: "omy3m_isaaclab_record",
    position: { x: 1160, y: 230 },
    data: {
      label: "IsaacLab 학습 데이터 저장",
      datasetPath: "isaaclab_datasets/omy3m",
      episodeName: "omy3m_demo_001",
      clientId: "isaacsim01",
      robotPrimPath: "/World/OMY3M_Learning/Robot",
      jointNames: "joint1,joint2,joint3,joint4,joint5,joint6",
      reward: 0,
      terminated: false,
      success: true,
      requireImage: true,
      saveFailed: false,
    },
  },
  {
    id: "omy3m-print",
    type: "print",
    position: { x: 1530, y: 230 },
    data: { label: "저장 결과 출력" },
  },
];

const omy3mTemplateEdges: Edge[] = [
  {
    id: "omy3m-e-prompt-action",
    source: "omy3m-prompt",
    target: "omy3m-action",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-action-execute",
    source: "omy3m-action",
    target: "omy3m-execute",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#f59e0b", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-prompt-record",
    source: "omy3m-prompt",
    target: "omy3m-record",
    sourceHandle: "text",
    targetHandle: "prompt",
    animated: true,
    style: { stroke: "#94a3b8", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-camera-record",
    source: "omy3m-camera",
    target: "omy3m-record",
    sourceHandle: "image",
    targetHandle: "image",
    animated: true,
    style: { stroke: "#60a5fa", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-state-record",
    source: "omy3m-state",
    target: "omy3m-record",
    sourceHandle: "state",
    targetHandle: "state",
    animated: true,
    style: { stroke: "#a78bfa", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-action-record",
    source: "omy3m-action",
    target: "omy3m-record",
    sourceHandle: "action",
    targetHandle: "action",
    animated: true,
    style: { stroke: "#f59e0b", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-execute-record",
    source: "omy3m-execute",
    target: "omy3m-record",
    sourceHandle: "result",
    targetHandle: "result",
    animated: true,
    style: { stroke: "#10b981", strokeWidth: 1.5 },
  },
  {
    id: "omy3m-e-record-print",
    source: "omy3m-record",
    target: "omy3m-print",
    sourceHandle: "dataset",
    targetHandle: "input",
    animated: true,
    style: { stroke: "#cbd5e1", strokeWidth: 1.5 },
  },
];

const fanucM710TemplateNodes = fanucM710Graph.nodes as Node[];
const fanucM710TemplateEdges = fanucM710Graph.edges as Edge[];
const fanucWorldObserveTemplateNodes = fanucObserveGraph.nodes as Node[];
const fanucWorldObserveTemplateEdges = fanucObserveGraph.edges as Edge[];

const fanucOmy3mTemplateNodes: Node[] = [
  {
    id: "fanuc-file",
    type: "dataset_upload",
    position: { x: 60, y: 120 },
    data: {
      label: "FANUC LS 파일 업로드",
      datasetPath: "",
      datasetFormat: "ls",
    },
  },
  {
    id: "fanuc-parse",
    type: "fanuc_ls_parse",
    position: { x: 390, y: 120 },
    data: { label: "FANUC LS 파싱" },
  },
  {
    id: "fanuc-retarget",
    type: "fanuc_omy3m_retarget",
    position: { x: 750, y: 120 },
    data: {
      label: "FANUC → OMY3M IK 리타게팅",
      positionScale: 0.25,
      targetOrigin: "0.35,0,0.25",
      targetAxisSigns: "1,-1,1",
      orientationOffsetDeg: "0,0,0",
      previewOnly: false,
      solveIk: true,
      requireAllIk: false,
      enableExecution: true,
      ikSeed: "0,-0.5,0.8,0,0.4,0",
      ikMaxIterations: 180,
      ikPositionToleranceM: 0.008,
      ikOrientationToleranceDeg: 8,
    },
  },
  {
    id: "fanuc-preview",
    type: "print",
    position: { x: 1140, y: 80 },
    data: { label: "변환 결과 확인" },
  },
  {
    id: "fanuc-execute",
    type: "robot_execute",
    position: { x: 1140, y: 260 },
    data: {
      label: "OMY3M 궤적 실행",
      executor: "robot-executor",
      mode: "omniverse",
      dryRun: false,
      robotType: "articulated_arm",
      controllerMode: "joint",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/OMY3M",
      omniverseStage: "/World",
      omniversePrimPath: "/World/OMY3M_Learning/Robot",
      omniverseCommand: "apply_omy3m_trajectory",
      omniverseTimeoutSec: 15,
      omniverseWaitForResponse: true,
    },
  },
  {
    id: "fanuc-dataset",
    type: "fanuc_omy3m_isaaclab_export",
    position: { x: 1510, y: 260 },
    data: {
      label: "IsaacLab 궤적 에피소드 저장",
      datasetPath: "isaaclab_datasets/omy3m_fanuc",
      episodeName: "fanuc_s05proc_001",
      instruction: "FANUC S05PROC trajectory retargeted to OMY3M",
      isaacLabBridgeRoot: "C:\\vlagraph_isaaclab_bridge",
      isaacLabTask: "Isaac-OMY3M-FANUC-Imitation-v0",
    },
  },
  {
    id: "fanuc-dataset-print",
    type: "print",
    position: { x: 1910, y: 260 },
    data: { label: "학습 데이터 경로" },
  },
];

const fanucOmy3mTemplateEdges: Edge[] = [
  { id: "fanuc-e-file-parse", source: "fanuc-file", target: "fanuc-parse", sourceHandle: "dataset", targetHandle: "dataset", animated: true },
  { id: "fanuc-e-parse-retarget", source: "fanuc-parse", target: "fanuc-retarget", sourceHandle: "program", targetHandle: "program", animated: true },
  { id: "fanuc-e-retarget-preview", source: "fanuc-retarget", target: "fanuc-preview", sourceHandle: "preview", targetHandle: "input", animated: true },
  { id: "fanuc-e-retarget-execute", source: "fanuc-retarget", target: "fanuc-execute", sourceHandle: "action", targetHandle: "action", animated: true },
  { id: "fanuc-e-retarget-dataset", source: "fanuc-retarget", target: "fanuc-dataset", sourceHandle: "action", targetHandle: "action", animated: true },
  { id: "fanuc-e-execute-dataset", source: "fanuc-execute", target: "fanuc-dataset", sourceHandle: "result", targetHandle: "result", animated: true },
  { id: "fanuc-e-dataset-print", source: "fanuc-dataset", target: "fanuc-dataset-print", sourceHandle: "dataset", targetHandle: "input", animated: true },
];

const GRAPH_TEMPLATES: Record<GraphTemplateId, GraphTemplate> = {
  openvla: {
    id: "openvla",
    label: "OPENVLA Graph",
    description: "Unity waypoint demo with OpenVLA inference.",
    nodes: initialNodes,
    edges: initialEdges,
  },
  gr00t: {
    id: "gr00t",
    label: "GR00T Graph",
    description: "Isaac Sim humanoid demo with NVIDIA GR00T inference.",
    nodes: gr00tTemplateNodes,
    edges: gr00tTemplateEdges,
  },
  omy3m: {
    id: "omy3m",
    label: "OMY3M 6DOF + IsaacLab Dataset",
    description: "Control OMY3M six-axis joints and save synchronized Isaac Lab training steps.",
    nodes: omy3mTemplateNodes,
    edges: omy3mTemplateEdges,
  },
  fanuc_m710: {
    id: "fanuc_m710",
    label: "FANUC M-710iC/20L 실행 검증 (Full)",
    description: "6단계로 표시합니다. 내부 검사를 유지하며 물리 예측이 준비되기 전에는 Full Guard가 실행을 차단합니다.",
    nodes: fanucM710TemplateNodes,
    edges: fanucM710TemplateEdges,
  },
  fanuc_m710_preflight: {
    id: "fanuc_m710_preflight",
    label: "FANUC M-710 LS Preflight (Dev)",
    description: "Parse LS and report trajectory constraints without sending motion.",
    nodes: fanucPreflightGraph.nodes as Node[],
    edges: fanucPreflightGraph.edges as Edge[],
  },
  fanuc_m710_snapshot: {
    id: "fanuc_m710_snapshot",
    label: "FANUC M-710 Physics Snapshot",
    description: "Read synchronized joints and physics clock without motion.",
    nodes: fanucSnapshotGraph.nodes as Node[],
    edges: fanucSnapshotGraph.edges as Edge[],
  },
  fanuc_m710_world_observe: {
    id: "fanuc_m710_world_observe",
    label: "FANUC M-710 World Observe",
    description: "Read M-710 joint state and work cell geometry without executing motion.",
    nodes: fanucWorldObserveTemplateNodes,
    edges: fanucWorldObserveTemplateEdges,
  },
  fanuc_omy3m: {
    id: "fanuc_omy3m",
    label: "FANUC LS → OMY3M Retarget",
    description: "Parse a FANUC LS program and preview a calibrated OMY3M trajectory before execution.",
    nodes: fanucOmy3mTemplateNodes,
    edges: fanucOmy3mTemplateEdges,
  },
  motion: {
    id: "motion",
    label: "Motion Learning Graph",
    description: "Upload a motion video and extract pose traces for imitation learning.",
    nodes: motionTemplateNodes,
    edges: motionTemplateEdges,
  },
  rgbd_follow: {
    id: "rgbd_follow",
    label: "RealSense RGB-D Follow",
    description: "D435i RGB-D pose capture into MediaPipe, depth fusion, and Isaac Sim humanoid retarget.",
    nodes: rgbdFollowTemplateNodes,
    edges: rgbdFollowTemplateEdges,
  },
  spatial_collection: {
    id: "spatial_collection",
    label: "Spatial Data Collection",
    description: "Scan Isaac Sim scene context and append spatial memory JSONL records.",
    nodes: spatialCollectionTemplateNodes,
    edges: spatialCollectionTemplateEdges,
  },
  spatial_rag: {
    id: "spatial_rag",
    label: "Spatial RAG Graph",
    description: "Retrieve spatial memory and inject it into a GR00T prompt before execution.",
    nodes: spatialRagTemplateNodes,
    edges: spatialRagTemplateEdges,
  },
  model_sync: {
    id: "model_sync",
    label: "Model Sync Graph",
    description: "Copy downloaded GR00T/Cosmos models into the Docker model volume.",
    nodes: modelSyncTemplateNodes,
    edges: modelSyncTemplateEdges,
  },
};

const GRAPH_TEMPLATE_OPTIONS = Object.values(GRAPH_TEMPLATES);

function cloneTemplateNodes(nodes: Node[]) {
  return nodes.map((node) => ({
    ...node,
    position: { ...node.position },
    data: JSON.parse(JSON.stringify(node.data ?? {})),
  }));
}

function cloneTemplateEdges(edges: Edge[]) {
  return edges.map((edge) => ({
    ...edge,
    style: edge.style ? { ...edge.style } : undefined,
  }));
}

function nextNodeIdFromNodes(nodes: Node[]) {
  const usedIds = new Set(nodes.map((node) => String(node.id)));
  let maxNumericSuffix = 0;

  nodes.forEach((node) => {
    const match = String(node.id).match(/^node-(\d+)$/);
    if (!match) return;

    const numericSuffix = Number(match[1]);
    if (Number.isFinite(numericSuffix)) {
      maxNumericSuffix = Math.max(maxNumericSuffix, numericSuffix);
    }
  });

  let candidate = maxNumericSuffix + 1;
  while (usedIds.has(`node-${candidate}`)) {
    candidate += 1;
  }

  return `node-${candidate}`;
}

export default function GraphPage() {
  const [graphTemplateId, setGraphTemplateId] = useState<GraphTemplateId>("openvla");
  const [pendingTemplateId, setPendingTemplateId] = useState<GraphTemplateId>("openvla");
  const [flowKey, setFlowKey] = useState("openvla");
  const [nodes, setNodes, onNodesChange] = useNodesState(cloneTemplateNodes(initialNodes).map((node) => ({
    ...node, data: { ...node.data, _graphId: "main", _graphLabel: GRAPH_TEMPLATES.openvla.label,
      _graphDescription: GRAPH_TEMPLATES.openvla.description },
  })));
  const [edges, setEdges, onEdgesChange] = useEdgesState(cloneTemplateEdges(initialEdges));
  const [selectedNode, setSelectedNode] = useState<Node | null>(null);
  const [activeGraphId, setActiveGraphId] = useState("main");
  const [loadOpen, setLoadOpen] = useState(false);
  const [bundleWindowId, setBundleWindowId] = useState<string | null>(null);
  const graphFileRef = useRef<HTMLInputElement>(null);
  const flowInstanceRef = useRef<ReactFlowInstance | null>(null);
  const [logs, setLogs] = useState<string[]>([]);
  const [runtimeReports, setRuntimeReports] = useState<Record<string, unknown>>({});
  const [storyEntries, setStoryEntries] = useState<StoryEntry[]>(IDLE_STORY);
  const [runOverview, setRunOverview] = useState<RunOverview>(IDLE_OVERVIEW);
  const [runtimeSessionId, setRuntimeSessionId] = useState<string | null>(null);
  const [runtimeStatus, setRuntimeStatus] = useState<string>("idle");
  const [isRunStarting, setIsRunStarting] = useState(false);
  const [isRunCancelling, setIsRunCancelling] = useState(false);
  const [isRecording, setIsRecording] = useState(false);
  const [recordingTargetNodeId, setRecordingTargetNodeId] = useState<string | null>(null);
  const [micStatusByNodeId, setMicStatusByNodeId] = useState<Record<string, string>>({});
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const mediaStreamRef = useRef<MediaStream | null>(null);
  const recordedChunksRef = useRef<Blob[]>([]);
  const editingLocked = isRunStarting || isRecording || (runtimeSessionId !== null && ACTIVE_RUNTIME_STATES.has(runtimeStatus));
  const editingLockedRef = useRef(editingLocked);
  editingLockedRef.current = editingLocked;
  const graphs = useMemo(() => graphList(nodes, edges), [nodes, edges]);
  useEffect(() => {
    if (graphs.length && !graphs.some((graph) => graph.id === activeGraphId)) setActiveGraphId(graphs[0].id);
  }, [graphs, activeGraphId]);
  useEffect(() => {
    const ids = new Set(nodes.map((node) => node.id));
    setEdges((prev) => prev.some((edge) => !ids.has(edge.source) || !ids.has(edge.target))
      ? prev.filter((edge) => ids.has(edge.source) && ids.has(edge.target)) : prev);
    if (!nodes.some((node) => node.type === "graph_bundle" && node.data.memberIds.some((id: string) => !ids.has(id)))) return;
    setNodes((prev) => prev.flatMap((node) => {
      if (node.type !== "graph_bundle") return [node];
      const memberIds = node.data.memberIds.filter((id: string) => ids.has(id));
      return memberIds.length ? [{ ...node, data: { ...node.data, memberIds } }] : [];
    }));
  }, [nodes, setNodes, setEdges]);
  const nodesRef = useRef(nodes);
  const edgesRef = useRef(edges);
  const unityStateClientIds = useMemo(() => {
    const ids = new Set<string>();
    nodes.forEach((node) => {
      if (node.type !== "unity_state") return;
      const clientId = String(node.data?.clientId ?? "default").trim() || "default";
      ids.add(clientId);
    });
    return Array.from(ids);
  }, [nodes]);
  const unityStateClientIdsKey = unityStateClientIds.join("|");
  const unityImagePreviewNodes = useMemo(
    () =>
      nodes
        .filter((node) => {
          const sourceMode = getSensorSourceMode(node.type, node.data?.sourceMode);
          if (sourceMode !== "remote_image" && sourceMode !== "fused") return false;
          return isUnityImagePreviewUrl(String(node.data?.remoteImageUrl ?? ""));
        })
        .map((node) => ({
          nodeId: node.id,
          url: String(node.data?.remoteImageUrl ?? "").trim(),
        })),
    [nodes]
  );
  const unityImagePreviewNodesKey = unityImagePreviewNodes
    .map((entry) => `${entry.nodeId}:${entry.url}`)
    .join("|");
  const unityImagePreviewEntries = useMemo(
    () => unityImagePreviewNodes.filter((entry) => entry.url),
    [unityImagePreviewNodesKey]
  );
  const rgbdPreviewNodes = useMemo(
    () =>
      nodes
        .filter(
          (node) =>
            node.type === "realsense_d435i_sensor" &&
            Boolean(node.data?.realsensePreviewEnabled)
        )
        .map((node) => ({
          nodeId: node.id,
          url: String(node.data?.remoteRgbdUrl ?? "").trim(),
          previewUrl: browserPreviewUrl(String(node.data?.remoteRgbdUrl ?? "").trim()),
        }))
        .filter((entry) => isHttpUrl(entry.url)),
    [nodes]
  );
  const rgbdPreviewNodesKey = rgbdPreviewNodes
    .map((entry) => `${entry.nodeId}:${entry.url}:${entry.previewUrl}`)
    .join("|");

  const setMicStatusForNode = useCallback((nodeId: string, status: string) => {
    setMicStatusByNodeId((prev) => ({ ...prev, [nodeId]: status }));
  }, []);

  const applyGraphTemplate = useCallback((templateId: GraphTemplateId) => {
    if (editingLockedRef.current) return;
    const template = GRAPH_TEMPLATES[templateId];
    setGraphTemplateId(templateId);
    setFlowKey(`${templateId}-${Date.now()}`);
    setNodes(cloneTemplateNodes(template.nodes).map((node) => ({ ...node,
      data: { ...node.data, _graphId: "main", _graphLabel: template.label, _graphDescription: template.description },
    })));
    setEdges(cloneTemplateEdges(template.edges));
    setSelectedNode(null);
    setActiveGraphId("main"); setBundleWindowId(null); setLoadOpen(false);
    setRuntimeSessionId(null);
    setRuntimeStatus("idle");
    setIsRunStarting(false);
    setIsRunCancelling(false);
    setStoryEntries(IDLE_STORY);
    setRunOverview(IDLE_OVERVIEW);
    setLogs([`Loaded template: ${template.label}`, template.description]);
    setRuntimeReports({});
    setMicStatusByNodeId({});
  }, [setEdges, setNodes]);

  const resetWorkspace = () => {
    setSelectedNode(null); setBundleWindowId(null); setRuntimeSessionId(null); setRuntimeStatus("idle");
    setIsRunStarting(false); setIsRunCancelling(false); setMicStatusByNodeId({});
    setStoryEntries(IDLE_STORY); setRunOverview(IDLE_OVERVIEW);
    setRuntimeReports({});
  };
  const newGraph = () => {
    if (editingLockedRef.current) return;
    resetWorkspace(); setNodes([]); setEdges([]); setActiveGraphId("main");
    setFlowKey(`blank-${workspaceId()}`); setLogs(["Created a new blank graph."]);
  };
  const addTemplate = () => {
    if (editingLockedRef.current) return;
    const template = GRAPH_TEMPLATES[pendingTemplateId];
    const id = `graph-${workspaceId()}`;
    const offset = nodes.length ? Math.max(...nodes.map((node) => node.position.x)) + 500 : 100;
    const added = instantiateGraph(cloneTemplateNodes(template.nodes), cloneTemplateEdges(template.edges), id,
      template.label, template.description, offset);
    setNodes((prev) => [...prev, ...added.nodes]); setEdges((prev) => [...prev, ...added.edges]);
    setGraphTemplateId(pendingTemplateId); setActiveGraphId(id); setLoadOpen(false); setFlowKey(`added-${id}`);
    setSelectedNode(null); setLogs((prev) => [`Added graph: ${template.label}`, ...prev]);
  };
  const createBundle = () => {
    if (editingLockedRef.current) return;
    try {
      const bundle = bundleGraph(nodes, edges, activeGraphId, `bundle-${workspaceId()}`);
      setNodes((prev) => [...prev.map((node) => ({ ...node, selected: false })), bundle]);
      setSelectedNode(null); setFlowKey(`bundled-${bundle.id}`);
      setLogs((prev) => [`Bundled ${bundle.data.label}: ${bundle.data.memberIds.length} nodes. Execution connections preserved.`, ...prev]);
    } catch (error) { setLogs((prev) => [String(error), ...prev]); }
  };
  const unbundle = useCallback((id: string) => {
    if (editingLockedRef.current) return;
    setNodes((prev) => prev.filter((node) => node.id !== id));
    setBundleWindowId(null); setSelectedNode(null); setFlowKey(`unbundle-${workspaceId()}`);
  }, [setNodes]);
  const saveGraph = () => {
    const url = URL.createObjectURL(new Blob([serializeWorkspace(nodes, edges)], { type: "application/json" }));
    const anchor = document.createElement("a"); anchor.href = url; anchor.download = "vlagraph-workspace.json";
    anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    setLogs((prev) => ["Saved workspace including bundle contents.", ...prev]);
  };
  const importGraph = async (file: File) => {
    if (editingLockedRef.current) return;
    try {
      const graph = parseWorkspace(await file.text(), Object.keys(nodeTypes));
      if (editingLockedRef.current) return;
      graph.nodes = graph.nodes.map((node) => ({ ...node, data: { ...node.data,
        _graphLabel: node.data._graphLabel || file.name.replace(/\.json$/i, ""),
      } }));
      resetWorkspace(); setNodes(graph.nodes); setEdges(graph.edges);
      setActiveGraphId(graph.nodes[0] ? graphId(graph.nodes[0]) : "main");
      setFlowKey(`import-${workspaceId()}`); setLoadOpen(false); setLogs([`Loaded workspace: ${file.name}`]);
    } catch (error) { setLogs((prev) => [`Load failed: ${String(error)}`, ...prev]); }
  };

  const stopMediaStream = useCallback(() => {
    if (!mediaStreamRef.current) return;

    mediaStreamRef.current.getTracks().forEach((track) => track.stop());
    mediaStreamRef.current = null;
  }, []);

  useEffect(() => {
    nodesRef.current = nodes;
  }, [nodes]);

  useEffect(() => {
    edgesRef.current = edges;
  }, [edges]);

  useEffect(() => {
    return () => {
      const recorder = mediaRecorderRef.current;
      if (recorder && recorder.state !== "inactive") {
        recorder.ondataavailable = null;
        recorder.onstop = null;
        recorder.stop();
      }

      stopMediaStream();
    };
  }, [stopMediaStream]);

  useEffect(() => {
    if (!selectedNode) return;

    const nextSelected = nodes.find((node) => node.id === selectedNode.id) ?? null;
    if (!nextSelected) {
      setSelectedNode(null);
      return;
    }

    if (nextSelected !== selectedNode) {
      setSelectedNode(nextSelected);
    }
  }, [nodes, selectedNode]);

  useEffect(() => {
    if (!unityStateClientIdsKey) return;

    let cancelled = false;
    const clientIds = unityStateClientIdsKey.split("|").filter(Boolean);
    const previewKeys = [
      "lastStateMessage",
      "lastStateScene",
      "lastStateObject",
      "lastStateSource",
      "lastStateGripper",
      "lastStatePose",
      "lastStatePreview",
    ] as const;

    const syncUnityStatePreviews = async () => {
      try {
        const entries = await Promise.all(
          clientIds.map(async (clientId) => {
            const response = await fetch(
              `${API_BASE}/api/unity/state?client_id=${encodeURIComponent(clientId)}`
            );

            if (!response.ok) {
              return [
                clientId,
                {
                  ok: false,
                  client_id: clientId,
                  message: `Failed to fetch Unity state (${response.status})`,
                },
              ] as const;
            }

            const payload = await response.json();
            return [clientId, payload] as const;
          })
        );

        if (cancelled) return;

        const payloadByClientId = Object.fromEntries(entries);

        setNodes((prev) =>
          prev.map((node) => {
            if (node.type !== "unity_state") {
              return node;
            }

            const clientId = String(node.data?.clientId ?? "default").trim() || "default";
            const previewPatch = buildRobotStateRuntimePreviewPatch(node, payloadByClientId[clientId]);
            const hasChanged = previewKeys.some(
              (key) => (node.data as Record<string, unknown> | undefined)?.[key] !== previewPatch[key]
            );

            if (!hasChanged) {
              return node;
            }

            return {
              ...node,
              data: {
                ...node.data,
                ...previewPatch,
              },
            };
          })
        );
      } catch {
        // Ignore background preview polling errors to keep editing responsive.
      }
    };

    void syncUnityStatePreviews();
    const intervalId = window.setInterval(() => {
      void syncUnityStatePreviews();
    }, 1500);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [unityStateClientIdsKey, setNodes]);

  useEffect(() => {
    if (!unityImagePreviewNodesKey) return;

    let cancelled = false;
    const previewEntries = unityImagePreviewEntries;

    const syncUnityImagePreviews = async () => {
      try {
        const entries = await Promise.all(
          previewEntries.map(async (entry) => {
            const response = await fetch(entry.url);
            if (!response.ok) {
              return [entry.nodeId, undefined] as const;
            }

            const contentType = String(response.headers.get("content-type") ?? "");
            if (contentType.includes("json")) {
              const payload = await response.json();
              return [entry.nodeId, extractImageFrameFromPayload(payload)] as const;
            }

            return [entry.nodeId, undefined] as const;
          })
        );

        if (cancelled) return;

        const imageByNodeId = Object.fromEntries(entries);
        setNodes((prev) =>
          prev.map((node) => {
            const nextFrame = imageByNodeId[node.id];
            if (typeof nextFrame !== "string" || !nextFrame) {
              return node;
            }

            const currentFrame = String((node.data as Record<string, unknown> | undefined)?.lastImageFrame ?? "");
            if (currentFrame === nextFrame) {
              return node;
            }

            return {
              ...node,
              data: {
                ...node.data,
                lastImageFrame: nextFrame,
              },
            };
          })
        );
      } catch {
        // Ignore background preview polling errors to keep editing responsive.
      }
    };

    void syncUnityImagePreviews();
    const intervalId = window.setInterval(() => {
      void syncUnityImagePreviews();
    }, 1000);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [unityImagePreviewEntries, unityImagePreviewNodesKey, setNodes]);

  useEffect(() => {
    if (!rgbdPreviewNodesKey || ACTIVE_RUNTIME_STATES.has(runtimeStatus)) return;

    let cancelled = false;
    let inFlight = false;
    const previewEntries = rgbdPreviewNodes;

    const syncRgbdPreviews = async () => {
      if (inFlight) return;
      inFlight = true;
      try {
        const entries = await Promise.all(
          previewEntries.map(async (entry) => {
            const response = await fetch(entry.previewUrl || entry.url, { cache: "no-store" });
            if (!response.ok) {
              return [entry.nodeId, {}] as const;
            }

            const payload = await response.json();
            return [entry.nodeId, buildRgbdPreviewPatchFromPayload(payload)] as const;
          })
        );

        if (cancelled) return;

        const patchByNodeId = Object.fromEntries(entries);
        setNodes((prev) =>
          prev.map((node) => {
            const patch = patchByNodeId[node.id];
            if (!patch || Object.keys(patch).length === 0) {
              return node;
            }

            return {
              ...node,
              data: {
                ...node.data,
                ...patch,
              },
            };
          })
        );
      } catch {
        // Keep the last decoded preview visible if one polling frame fails.
      } finally {
        inFlight = false;
      }
    };

    void syncRgbdPreviews();
    const intervalId = window.setInterval(() => {
      void syncRgbdPreviews();
    }, RGBD_PREVIEW_POLL_INTERVAL_MS);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [rgbdPreviewNodesKey, runtimeStatus, setNodes]);

  const applyRuntimeSnapshot = useCallback((snapshot: any, sourceNodes?: Node[]) => {
    const liveNodes = (sourceNodes ?? nodesRef.current).filter((node) => node.type !== "graph_bundle");
    const liveEdges = edgesRef.current;
    const sessionState = String(snapshot?.status ?? "idle");
    const presentation = buildRuntimePresentation(liveNodes, snapshot);
    const context = presentation.context ?? {};
    setRuntimeReports(context);
    const summaryByNodeId = new Map(
      presentation.story
        .filter((entry) => entry.nodeId)
        .map((entry) => [String(entry.nodeId), entry])
    );
    const activeNodeId = String(snapshot?.active_node_id ?? "");
    const runtimeMessage = String(snapshot?.error ?? snapshot?.message ?? "");

    setRuntimeStatus(sessionState);
    setRunOverview(presentation.overview);
    setStoryEntries(presentation.story.length > 0 ? presentation.story : IDLE_STORY);
    setLogs(presentation.logs.length > 0 ? presentation.logs : ["Runtime session ready"]);
    setIsRunStarting(false);
    if (sessionState !== "cancelling") {
      setIsRunCancelling(false);
    }

    setNodes((prev) =>
      prev.map((node) => {
        const entry = summaryByNodeId.get(node.id);
        const isActiveNode = activeNodeId === node.id && ACTIVE_RUNTIME_STATES.has(sessionState);

        if (entry) {
          return {
            ...node,
            data: {
              ...node.data,
              status: isActiveNode ? "running" : entry.state === "warning" ? "error" : entry.state,
              lastSummary: isActiveNode ? "실행 중" : entry.summary,
              lastDetail: isActiveNode
                ? runtimeMessage || "현재 노드를 처리 중입니다."
                : entry.detail,
              ...buildSensorRuntimePreviewPatch(node, context[node.id]),
              ...buildRgbdVisionRuntimePreviewPatch(node, context[node.id]),
              ...buildRobotStateRuntimePreviewPatch(node, context[node.id]),
              ...buildVideoMotionRuntimePreviewPatch(node, context, liveNodes, liveEdges),
              ...buildMotionExtractRuntimePatch(node, context[node.id]),
              ...buildFanucIsaacLabRuntimePatch(node, context[node.id]),
            },
          };
        }

        if (ACTIVE_RUNTIME_STATES.has(sessionState)) {
          return {
            ...node,
            data: {
              ...node.data,
              status: isActiveNode ? "running" : "running",
              lastSummary: runtimeStatusLabel(sessionState),
              lastDetail: runtimeMessage || "런타임 세션이 노드를 순차적으로 처리하고 있습니다.",
              ...buildSensorRuntimePreviewPatch(node, context[node.id]),
              ...buildRgbdVisionRuntimePreviewPatch(node, context[node.id]),
              ...buildRobotStateRuntimePreviewPatch(node, context[node.id]),
              ...buildVideoMotionRuntimePreviewPatch(node, context, liveNodes, liveEdges),
              ...buildMotionExtractRuntimePatch(node, context[node.id]),
              ...buildFanucIsaacLabRuntimePatch(node, context[node.id]),
            },
          };
        }

        if (sessionState === "cancelled") {
          return {
            ...node,
            data: {
              ...node.data,
              status: "idle",
              lastSummary: "실행 취소",
              lastDetail: runtimeMessage || "취소 요청에 따라 런타임이 종료되었습니다.",
              ...buildSensorRuntimePreviewPatch(node, context[node.id]),
              ...buildRgbdVisionRuntimePreviewPatch(node, context[node.id]),
              ...buildRobotStateRuntimePreviewPatch(node, context[node.id]),
              ...buildVideoMotionRuntimePreviewPatch(node, context, liveNodes, liveEdges),
              ...buildMotionExtractRuntimePatch(node, context[node.id]),
              ...buildFanucIsaacLabRuntimePatch(node, context[node.id]),
            },
          };
        }

        if (sessionState === "error") {
          return {
            ...node,
            data: {
              ...node.data,
              status: "error",
              lastSummary: "런타임 오류",
              lastDetail: runtimeMessage || "런타임이 오류로 중지되었습니다.",
              ...buildSensorRuntimePreviewPatch(node, context[node.id]),
              ...buildRgbdVisionRuntimePreviewPatch(node, context[node.id]),
              ...buildRobotStateRuntimePreviewPatch(node, context[node.id]),
              ...buildVideoMotionRuntimePreviewPatch(node, context, liveNodes, liveEdges),
              ...buildMotionExtractRuntimePatch(node, context[node.id]),
              ...buildFanucIsaacLabRuntimePatch(node, context[node.id]),
            },
          };
        }

        return {
          ...node,
          data: {
            ...node.data,
            status: sessionState === "completed" ? "idle" : node.data?.status ?? "idle",
            ...buildSensorRuntimePreviewPatch(node, context[node.id]),
            ...buildRgbdVisionRuntimePreviewPatch(node, context[node.id]),
            ...buildRobotStateRuntimePreviewPatch(node, context[node.id]),
            ...buildVideoMotionRuntimePreviewPatch(node, context, liveNodes, liveEdges),
            ...buildMotionExtractRuntimePatch(node, context[node.id]),
            ...buildFanucIsaacLabRuntimePatch(node, context[node.id]),
          },
        };
      })
    );
  }, [setNodes]);

  useEffect(() => {
    if (!runtimeSessionId || !ACTIVE_RUNTIME_STATES.has(runtimeStatus)) {
      return;
    }

    let cancelled = false;
    let timeoutId: number | undefined;

    const poll = async () => {
      try {
        const snapshot = (await getGraphRuntime(runtimeSessionId)) as any;
        if (cancelled) return;

        applyRuntimeSnapshot(snapshot);

        const nextStatus = String(snapshot?.status ?? "idle");
        if (FINAL_RUNTIME_STATES.has(nextStatus)) {
          return;
        }

        timeoutId = window.setTimeout(() => {
          void poll();
        }, runtimePollIntervalMs(nodesRef.current));
      } catch (error) {
        if (cancelled) return;

        const message = error instanceof Error ? error.message : String(error);
        setRuntimeStatus("error");
        setIsRunStarting(false);
        setIsRunCancelling(false);
        setRunOverview({
          state: "error",
          title: "그래프 런타임 상태 조회 실패",
          detail: "백엔드에서 세션 상태를 가져오지 못했습니다.",
          metrics: [
            {
              label: "상태",
              value: toneLabel("error"),
              detail: message,
              tone: "error",
            },
            {
              label: "실행 경로",
              value: `${nodesRef.current.length} 단계`,
              detail: nodesRef.current
                .map((node) => String(node.data?.label ?? humanizeNodeType(node.type)))
                .join(" -> "),
              tone: "idle",
            },
            {
              label: "현재 단계",
              value: "상태 조회 실패",
              detail: "세션이 계속 실행 중인지 확인이 필요합니다.",
              tone: "error",
            },
            {
              label: "반복 상태",
              value: runtimeSessionId,
              detail: "세션 ID 기준으로 백엔드 상태를 다시 확인해 주세요.",
              tone: "error",
            },
          ],
        });
        setStoryEntries([
          {
            id: "runtime-poll-failed",
            state: "error",
            title: "런타임 상태를 갱신하지 못했습니다.",
            detail: message,
            summary: "상태 조회 실패",
          },
        ]);
        setLogs([`Runtime polling failed: ${message}`]);
      }
    };

    timeoutId = window.setTimeout(() => {
      void poll();
    }, Math.min(100, runtimePollIntervalMs(nodesRef.current)));

    return () => {
      cancelled = true;
      if (timeoutId !== undefined) {
        window.clearTimeout(timeoutId);
      }
    };
  }, [applyRuntimeSnapshot, runtimeSessionId, runtimeStatus]);

  const removeNode = useCallback(
    (nodeId: string) => {
      if (editingLockedRef.current) return;
      if (recordingTargetNodeId === nodeId) {
        const recorder = mediaRecorderRef.current;
        if (recorder && recorder.state !== "inactive") {
          recorder.stop();
        }
      }

      setNodes((prev) => prev.filter((node) => node.id !== nodeId));
      setEdges((prev) => prev.filter((edge) => edge.source !== nodeId && edge.target !== nodeId));
      setSelectedNode((prev) => (prev?.id === nodeId ? null : prev));
    },
    [recordingTargetNodeId, setNodes, setEdges]
  );

  const toggleRealSenseBridge = useCallback(
    async (nodeId: string) => {
      const node = nodesRef.current.find((entry) => entry.id === nodeId);
      if (!node || node.type !== "realsense_d435i_sensor") {
        return;
      }

      const enabled = Boolean(node.data?.realsensePreviewEnabled);
      const remoteUrl = String(node.data?.remoteRgbdUrl ?? "").trim();
      const port = realsensePortFromUrl(remoteUrl);

      const patchNode = (patch: Record<string, unknown>) => {
        setNodes((prev) =>
          prev.map((entry) =>
            entry.id === nodeId
              ? {
                  ...entry,
                  data: {
                    ...entry.data,
                    ...patch,
                  },
                }
              : entry
          )
        );
        setSelectedNode((prev) =>
          prev?.id === nodeId
            ? {
                ...prev,
                data: {
                  ...prev.data,
                  ...patch,
                },
              }
            : prev
        );
      };

      if (enabled) {
        patchNode({
          realsensePreviewEnabled: false,
          realsenseBridgeStatus: "stopping",
          realsenseBridgeMessage: "Stopping preview and managed bridge...",
        });
        try {
          const result = (await stopRealSenseBridge()) as any;
          const reachable = Boolean(result?.reachable);
          patchNode({
            realsenseBridgeStatus: reachable ? "external" : "off",
            realsenseBridgeMessage:
              String(result?.message ?? "") ||
              (reachable ? "External bridge is still reachable." : "RealSense bridge stopped."),
          });
          setLogs((prev) => [`RealSense off: ${String(result?.message ?? "preview stopped")}`, ...prev]);
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          patchNode({
            realsenseBridgeStatus: "error",
            realsenseBridgeMessage: message,
          });
          setLogs((prev) => [`RealSense stop failed: ${message}`, ...prev]);
        }
        return;
      }

      patchNode({
        realsenseBridgeStatus: "starting",
        realsenseBridgeMessage: "Starting bridge...",
      });
      try {
        const status = (await getRealSenseBridgeStatus(port)) as any;
        const result = status?.reachable
          ? status
          : ((await startRealSenseBridge({
              port,
              width: Number(node.data?.captureWidth ?? 640) || 640,
              height: Number(node.data?.captureHeight ?? 480) || 480,
              fps: Number(node.data?.fps ?? 30) || 30,
              serial: String(node.data?.realsenseSerial ?? "").trim(),
              mockWhenUnavailable: Boolean(node.data?.mockWhenUnavailable ?? false),
            })) as any);
        const reachable = Boolean(result?.reachable);
        patchNode({
          realsensePreviewEnabled: reachable,
          realsenseBridgeStatus: reachable ? "on" : "error",
          realsenseBridgeMessage:
            String(result?.message ?? "") ||
            (reachable ? "RealSense bridge is reachable." : String(result?.error ?? "Bridge is not reachable.")),
        });
        setLogs((prev) => [
          reachable
            ? `RealSense on: ${String(result?.frame_url ?? (remoteUrl || "bridge reachable"))}`
            : `RealSense start failed: ${String(result?.error ?? "bridge is not reachable")}`,
          ...prev,
        ]);
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        patchNode({
          realsensePreviewEnabled: false,
          realsenseBridgeStatus: "error",
          realsenseBridgeMessage: message,
        });
        setLogs((prev) => [`RealSense start failed: ${message}`, ...prev]);
      }
    },
    [setNodes]
  );

  const isValidConnection = useCallback(
    (params: Connection | Edge) => {
      const sourceNode = nodes.find((n) => n.id === params.source);
      const targetNode = nodes.find((n) => n.id === params.target);

      return canConnect({
        sourceNodeType: sourceNode?.type,
        sourceHandle: params.sourceHandle,
        targetNodeType: targetNode?.type,
        targetHandle: params.targetHandle,
      });
    },
    [nodes]
  );

 const onConnect = useCallback(
  (params: Edge | Connection) => {
    const sourceNode = nodes.find((n) => n.id === params.source);
    const targetNode = nodes.find((n) => n.id === params.target);

    const ok = isValidConnection(params);

    if (!ok) {
      setLogs((prev) => [
        `Invalid connection: ${String(sourceNode?.type)}:${String(params.sourceHandle)} -> ${String(targetNode?.type)}:${String(params.targetHandle)}`,
        ...prev,
      ]);
      return;
    }

    const sourcePinType = pinTypeOfNodeHandle(
      sourceNode?.type ?? "",
      params.sourceHandle,
      "source"
    );

    const targetPinType = pinTypeOfNodeHandle(
      targetNode?.type ?? "",
      params.targetHandle,
      "target"
    );

    const edgeType = edgeKindFromPins(sourcePinType, targetPinType);

    setEdges((eds) =>
      addEdge(
        {
          ...params,
          type: edgeType,
          animated: true,
          style: { stroke: "#94a3b8", strokeWidth: 1.5 },
        },
        eds
      )
    );
  },
  [isValidConnection, nodes, setEdges]
);

  const onAddNode = useCallback(
    (type: string) => {
      if (editingLockedRef.current || bundleWindowId) return;
      if (nodesRef.current.some((node) => node.type === "graph_bundle" && graphId(node) === activeGraphId)) {
        setLogs((prev) => ["Unbundle the selected graph before adding nodes.", ...prev]); return;
      }
      setNodes((prev) => {
        const nextId = nextNodeIdFromNodes(prev);
        const defaults = createNodeData(type);
        const newNode: Node = {
          id: nextId,
          type,
          position: { x: 200 + prev.length * 40, y: 120 + prev.length * 30 },
          data: createNodeData(type, { label: defaults.label ?? type,
            _graphId: activeGraphId, _graphLabel: graphs.find((graph) => graph.id === activeGraphId)?.label || "Manual Graph" }),
        };

        return [...prev, newNode];
      });
    },
    [setNodes, bundleWindowId, activeGraphId, graphs]
  );

  const onNodeDataChange = useCallback((nodeId: string, key: string, value: unknown) => {
    if (editingLockedRef.current) return;
    setNodes((prev) =>
      prev.map((node) => {
        if (node.id !== nodeId) return node;

        let parsed: unknown = value;

        if (typeof value === "string") {
          const trimmed = value.trim();

          if (
            (trimmed.startsWith("{") && trimmed.endsWith("}")) ||
            (trimmed.startsWith("[") && trimmed.endsWith("]"))
          ) {
            try {
              parsed = JSON.parse(trimmed);
            } catch {
              parsed = value;
            }
          } else if (trimmed === "true" || trimmed === "false") {
            parsed = trimmed === "true";
          } else if (!Number.isNaN(Number(trimmed)) && trimmed !== "") {
            parsed = Number(trimmed);
          }
        }

        return { ...node, data: { ...node.data, [key]: parsed } };
      })
    );

    setSelectedNode((prev) => {
      if (!prev || prev.id !== nodeId) return prev;
      return { ...prev, data: { ...prev.data, [key]: value } };
    });
  }, [setNodes]);

  const linkAudioFileToSpeechNode = useCallback((
    nodeId: string,
    result: any,
    source: "upload" | "recording"
  ) => {
    onNodeDataChange(nodeId, "audioPath", result.file_path);
    setLogs((prev) => [
      `${source === "recording" ? "Recorded audio linked" : "Audio linked"}: ${result.file_name}`,
      ...prev,
    ]);
  }, [onNodeDataChange]);

  const onRun = useCallback(async () => {
    if (runtimeSessionId && ACTIVE_RUNTIME_STATES.has(runtimeStatus)) {
      return;
    }

    setIsRunStarting(true);
    setIsRunCancelling(false);
    setRuntimeReports({});
    try {
      const execution = executionGraph(nodes, edges);
      if (!execution.nodes.length) throw new Error("실행할 노드가 없습니다. Load 또는 Add Node로 그래프를 구성하세요.");
      setRunOverview({
        state: "running",
        title: "그래프 런타임 시작 중",
        detail: "그래프를 서버 메모리에 올리고 외부 실행기와 왕복할 세션을 준비하고 있습니다.",
        metrics: [
          {
            label: "상태",
            value: toneLabel("running"),
            detail: `${nodes.length}개 노드 런타임을 초기화하고 있습니다.`,
            tone: "running",
          },
          {
            label: "실행 경로",
            value: `${nodes.length} 단계`,
            detail: nodes.map((node) => String(node.data?.label ?? humanizeNodeType(node.type))).join(" -> "),
            tone: "idle",
          },
          {
            label: "현재 단계",
            value: "메모리 적재 중",
            detail: "그래프 세션 ID를 발급하고 실행 준비를 마치고 있습니다.",
            tone: "running",
          },
          {
            label: "반복 상태",
            value: "세션 준비",
            detail: "Unity 피드백 루프 조건이 맞으면 반복 실행으로 전환됩니다.",
            tone: "running",
          },
        ],
      });
      setStoryEntries([
        {
          id: "run-start",
          state: "running",
          title: "그래프 런타임을 시작합니다.",
          detail: `${nodes.length}개 노드와 ${edges.length}개 연결을 서버 메모리에 올리고 있습니다.`,
          summary: "세션 시작",
        },
      ]);
      setLogs(["Starting graph runtime..."]);
      setNodes((prev) =>
        prev.map((node) => ({
          ...node,
          data: {
            ...node.data,
            status: "running",
            lastSummary: "실행 중",
            lastDetail: "백엔드 응답을 기다리고 있습니다.",
            ...(node.type === "video_motion_input" ? { lastPosePreview: undefined } : {}),
            ...buildSensorRuntimePreviewPatch(node, null),
            ...buildRgbdVisionRuntimePreviewPatch(node, null),
            ...buildRobotStateRuntimePreviewPatch(node, null),
          },
        }))
      );

      const snapshot = (await startGraphRuntime({
        nodes: execution.nodes,
        edges: execution.edges,
        options: runtimeOptionsForGraph(execution.nodes),
      })) as any;
      setRuntimeSessionId(String(snapshot?.session_id ?? ""));
      applyRuntimeSnapshot(snapshot, execution.nodes);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      setRuntimeSessionId(null);
      setRuntimeStatus("error");
      setRunOverview({
        state: "error",
        title: "그래프 런타임 시작 실패",
        detail: "백엔드 요청 또는 세션 생성 단계에서 실행이 중단되었습니다.",
        metrics: [
          {
            label: "상태",
            value: toneLabel("error"),
            detail: message,
            tone: "error",
          },
          {
            label: "실행 경로",
            value: `${nodes.length} 단계`,
            detail: nodes.map((node) => String(node.data?.label ?? humanizeNodeType(node.type))).join(" -> "),
            tone: "idle",
          },
          {
            label: "현재 단계",
            value: "세션 생성 실패",
            detail: "런타임 세션을 만들지 못했습니다.",
            tone: "error",
          },
          {
            label: "반복 상태",
            value: "시작 불가",
            detail: message,
            tone: "error",
          },
        ],
      });
      setStoryEntries([
        {
          id: "run-failed",
          state: "error",
          title: "그래프 런타임 시작에 실패했습니다.",
          detail: message,
          summary: "요청 실패",
        },
      ]);
      setLogs([`Runtime start failed: ${message}`]);
      setNodes((prev) =>
        prev.map((node) => ({
          ...node,
          data: {
            ...node.data,
            status: "error",
            lastSummary: "요청 실패",
            lastDetail: message,
          },
        }))
      );
    } finally {
      setIsRunStarting(false);
    }
  }, [applyRuntimeSnapshot, edges, nodes, runtimeSessionId, runtimeStatus]);

  const onCancelRun = useCallback(async () => {
    if (!runtimeSessionId || !ACTIVE_RUNTIME_STATES.has(runtimeStatus) || isRunCancelling) {
      return;
    }

    setIsRunCancelling(true);
    try {
      const snapshot = (await cancelGraphRuntime(runtimeSessionId)) as any;
      applyRuntimeSnapshot(snapshot);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      setIsRunCancelling(false);
      setRunOverview({
        state: "error",
        title: "그래프 런타임 취소 실패",
        detail: "백엔드에 취소 요청을 전달하지 못했습니다.",
        metrics: [
          {
            label: "상태",
            value: toneLabel("error"),
            detail: message,
            tone: "error",
          },
          {
            label: "실행 경로",
            value: `${nodes.length} 단계`,
            detail: nodes.map((node) => String(node.data?.label ?? humanizeNodeType(node.type))).join(" -> "),
            tone: "idle",
          },
          {
            label: "현재 단계",
            value: "취소 요청 실패",
            detail: "세션이 계속 실행 중일 수 있어 추가 확인이 필요합니다.",
            tone: "error",
          },
          {
            label: "반복 상태",
            value: runtimeSessionId,
            detail: "세션 ID로 백엔드 상태를 다시 확인해 주세요.",
            tone: "error",
          },
        ],
      });
      setStoryEntries([
        {
          id: "runtime-cancel-failed",
          state: "error",
          title: "그래프 런타임 취소 요청에 실패했습니다.",
          detail: message,
          summary: "취소 실패",
        },
      ]);
      setLogs([`Runtime cancel failed: ${message}`]);
    }
  }, [applyRuntimeSnapshot, isRunCancelling, nodes, runtimeSessionId, runtimeStatus]);

  const uploadDatasetForNode = useCallback(async (nodeId: string, file: File) => {
    try {
      setLogs((prev) => ["Uploading dataset...", ...prev]);
      const result = (await uploadFile(file)) as any;
      onNodeDataChange(nodeId, "datasetPath", result.file_path);
      setLogs((prev) => [`Uploaded: ${result.file_name}`, ...prev]);
    } catch (error) {
      setLogs((prev) => [`Upload failed: ${error instanceof Error ? error.message : String(error)}`, ...prev]);
    }
  }, [onNodeDataChange]);

  const uploadInitialSettingsForNode = useCallback(async (nodeId: string, file: File) => {
    try {
      setLogs((prev) => ["Uploading FANUC initial LS...", ...prev]);
      const result = (await uploadFile(file)) as any;
      onNodeDataChange(nodeId, "initialSettingsPath", result.file_path);
      onNodeDataChange(nodeId, "initialSettingsFormat", "ls");
      setLogs((prev) => [`Uploaded initial LS: ${result.file_name}`, ...prev]);
    } catch (error) {
      setLogs((prev) => [`Initial LS upload failed: ${error instanceof Error ? error.message : String(error)}`, ...prev]);
    }
  }, [onNodeDataChange]);

  const uploadImageForNode = useCallback(async (nodeId: string, file: File) => {
    try {
      setLogs((prev) => ["Uploading image for VLA input...", ...prev]);
      const result = (await uploadFile(file)) as any;
      const filePath = String(result.file_path ?? "");
      const fileName = filePath.split("/").pop() || result.file_name || file.name;
      const imageUrl = `${API_BASE}/uploads/${fileName}`;
      onNodeDataChange(nodeId, "imageUrl", imageUrl);
      setLogs((prev) => [`Image linked for real/stub inference: ${fileName}`, ...prev]);
    } catch (error) {
      setLogs((prev) => [`Image upload failed: ${error instanceof Error ? error.message : String(error)}`, ...prev]);
    }
  }, [onNodeDataChange]);

  const uploadVideoForNode = useCallback(async (nodeId: string, file: File) => {
    try {
      setLogs((prev) => ["Uploading motion video...", ...prev]);
      const result = (await uploadFile(file)) as any;
      const filePath = String(result.file_path ?? "");
      const fileName = filePath.split(/[\\/]/).pop() || result.file_name || file.name;
      const videoUrl = `${API_BASE}/uploads/${fileName}`;
      onNodeDataChange(nodeId, "videoPath", videoUrl);
      setLogs((prev) => [`Motion video uploaded: ${fileName}`, ...prev]);
    } catch (error) {
      setLogs((prev) => [`Motion video upload failed: ${error instanceof Error ? error.message : String(error)}`, ...prev]);
    }
  }, [onNodeDataChange]);

  const uploadAudioForNode = useCallback(async (nodeId: string, file: File) => {
    try {
      setLogs((prev) => ["Uploading audio...", ...prev]);
      const result = (await uploadFile(file)) as any;
      linkAudioFileToSpeechNode(nodeId, result, "upload");
      setMicStatusForNode(nodeId, "Audio uploaded");
    } catch (error) {
      setLogs((prev) => [`Audio upload failed: ${error instanceof Error ? error.message : String(error)}`, ...prev]);
      setMicStatusForNode(nodeId, "Audio upload failed");
    }
  }, [linkAudioFileToSpeechNode, setMicStatusForNode]);

  const patchNodeData = useCallback((nodeId: string, patch: Record<string, unknown>) => {
    setNodes((prev) =>
      prev.map((node) =>
        node.id === nodeId
          ? {
              ...node,
              data: {
                ...node.data,
                ...patch,
              },
            }
          : node
      )
    );

    setSelectedNode((prev) =>
      prev?.id === nodeId
        ? {
            ...prev,
            data: {
              ...prev.data,
              ...patch,
            },
          }
        : prev
    );
  }, [setNodes]);

  const initIsaacGr00tForNode = useCallback(async (nodeId: string) => {
    const node = nodesRef.current.find((candidate) => candidate.id === nodeId);
    if (!node || node.type !== "unity_state") {
      return;
    }

    const clientId = String(node.data?.clientId ?? "isaacsim01").trim() || "isaacsim01";
    const omniverseEndpoint =
      String(node.data?.omniverseEndpoint ?? "ws://host.docker.internal:9900/omniverse").trim() ||
      "ws://host.docker.internal:9900/omniverse";
    const timeoutSec = Number(node.data?.initTimeoutSec ?? node.data?.omniverseTimeoutSec ?? 15);

    patchNodeData(nodeId, {
      initStatus: "running",
      initMessage: "Sending init command to Isaac Sim...",
      initEndpoint: omniverseEndpoint,
      initPrimPath: "",
      status: "running",
      lastSummary: "Isaac GR00T init",
      lastDetail: "Isaac Sim extension 응답을 기다리고 있습니다.",
    });
    setLogs((prev) => [`Isaac GR00T init requested: ${omniverseEndpoint}`, ...prev]);

    try {
      const response = (await initIsaacGr00tScene({
        clientId,
        omniverseEndpoint,
        timeoutSec: Number.isFinite(timeoutSec) && timeoutSec > 0 ? timeoutSec : 15,
      })) as any;
      const ok = Boolean(response?.ok);
      const message = String(
        response?.message ?? (ok ? "Isaac GR00T scene initialized." : "Isaac GR00T scene init failed.")
      );
      const primPath = String(response?.prim_path ?? response?.primPath ?? "");

      patchNodeData(nodeId, {
        initStatus: ok ? "success" : "error",
        initMessage: message,
        initEndpoint: String(response?.endpoint ?? omniverseEndpoint),
        initPrimPath: primPath,
        status: ok ? "success" : "error",
        lastSummary: ok ? "Init 완료" : "Init 실패",
        lastDetail: message,
      });
      setLogs((prev) => [
        `${ok ? "Isaac GR00T init ok" : "Isaac GR00T init failed"}: ${message}`,
        ...prev,
      ]);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      patchNodeData(nodeId, {
        initStatus: "error",
        initMessage: message,
        initEndpoint: omniverseEndpoint,
        status: "error",
        lastSummary: "Init 요청 실패",
        lastDetail: message,
      });
      setLogs((prev) => [`Isaac GR00T init request failed: ${message}`, ...prev]);
    }
  }, [patchNodeData]);

  const onStartRecording = useCallback(async (nodeId: string) => {
    if (isRecording) return;

    if (typeof window === "undefined" || typeof MediaRecorder === "undefined") {
      setMicStatusForNode(nodeId, "Microphone not supported");
      setLogs((prev) => ["Microphone recording is not supported in this browser.", ...prev]);
      return;
    }

    if (!navigator.mediaDevices?.getUserMedia) {
      setMicStatusForNode(nodeId, "Microphone API unavailable");
      setLogs((prev) => ["Microphone access is unavailable in this environment.", ...prev]);
      return;
    }

    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const mimeType = pickSupportedAudioMimeType();
      const recorder = mimeType
        ? new MediaRecorder(stream, { mimeType })
        : new MediaRecorder(stream);

      mediaStreamRef.current = stream;
      mediaRecorderRef.current = recorder;
      recordedChunksRef.current = [];
      setRecordingTargetNodeId(nodeId);

      recorder.ondataavailable = (event) => {
        if (event.data && event.data.size > 0) {
          recordedChunksRef.current.push(event.data);
        }
      };

      recorder.onstop = async () => {
        const finalMimeType = recorder.mimeType || mimeType || "audio/webm";
        const audioBlob = new Blob(recordedChunksRef.current, { type: finalMimeType });

        mediaRecorderRef.current = null;
        recordedChunksRef.current = [];
        stopMediaStream();
        setIsRecording(false);
        setRecordingTargetNodeId(null);

        if (audioBlob.size === 0) {
          setMicStatusForNode(nodeId, "No audio captured");
          setLogs((prev) => ["Recording finished, but no audio data was captured.", ...prev]);
          return;
        }

        try {
          setMicStatusForNode(nodeId, "Uploading recording...");
          const extension = fileExtensionFromMimeType(finalMimeType);
          const audioFile = new File(
            [audioBlob],
            `mic-recording-${Date.now()}.${extension}`,
            { type: finalMimeType }
          );
          const result = (await uploadFile(audioFile)) as any;
          linkAudioFileToSpeechNode(nodeId, result, "recording");
          setMicStatusForNode(nodeId, "Recording uploaded");
        } catch (error) {
          setMicStatusForNode(nodeId, "Recording upload failed");
          setLogs((prev) => [
            `Recording upload failed: ${error instanceof Error ? error.message : String(error)}`,
            ...prev,
          ]);
        }
      };

      recorder.onerror = () => {
        setIsRecording(false);
        setRecordingTargetNodeId(null);
        setMicStatusForNode(nodeId, "Recording failed");
        stopMediaStream();
        mediaRecorderRef.current = null;
        recordedChunksRef.current = [];
        setLogs((prev) => ["Microphone recording failed.", ...prev]);
      };

      recorder.start();
      setIsRecording(true);
      setMicStatusForNode(nodeId, "Recording...");
      setLogs((prev) => ["Microphone recording started.", ...prev]);
    } catch (error) {
      setIsRecording(false);
      setRecordingTargetNodeId(null);
      setMicStatusForNode(nodeId, "Microphone permission denied");
      stopMediaStream();
      setLogs((prev) => [
        `Unable to access microphone: ${error instanceof Error ? error.message : String(error)}`,
        ...prev,
      ]);
    }
  }, [isRecording, linkAudioFileToSpeechNode, setMicStatusForNode, stopMediaStream]);

  const onStopRecording = useCallback(() => {
    const recorder = mediaRecorderRef.current;
    if (!recorder || recorder.state === "inactive") return;

    if (recordingTargetNodeId) {
      setMicStatusForNode(recordingTargetNodeId, "Stopping recording...");
    }
    recorder.stop();
  }, [recordingTargetNodeId, setMicStatusForNode]);

  const renderedNodes = useMemo(() => {
    return nodes.map((node) => ({
      ...node,
      data: {
        ...node.data,
        onRemove: removeNode,
        onNodeDataChange,
        ...(node.type === "graph_bundle" ? {
          edgeCount: edges.filter((edge) => node.data.memberIds.includes(edge.source) && node.data.memberIds.includes(edge.target)).length,
          onShowNodes: () => { setBundleWindowId(node.id); setSelectedNode(null); },
          onUnbundle: () => unbundle(node.id), locked: editingLocked,
        } : {}),
        ...(node.type === "dataset_upload"
          ? {
              onDatasetUpload: (file: File) => uploadDatasetForNode(node.id, file),
              onInitialSettingsUpload: (file: File) => uploadInitialSettingsForNode(node.id, file),
            }
          : {}),
        ...(node.type === "image_input"
          ? {
              onImageUpload: (file: File) => uploadImageForNode(node.id, file),
            }
          : {}),
        ...(node.type === "video_motion_input"
          ? {
              onVideoUpload: (file: File) => uploadVideoForNode(node.id, file),
            }
          : {}),
        ...(node.type === "unity_state"
          ? {
              onIsaacGr00tInit: () => initIsaacGr00tForNode(node.id),
            }
          : {}),
        ...(node.type === "realsense_d435i_sensor"
          ? {
              onToggleRealSenseBridge: () => toggleRealSenseBridge(node.id),
            }
          : {}),
        ...(node.type === "speech_to_text"
          ? {
              onAudioUpload: (file: File) => uploadAudioForNode(node.id, file),
              onStartMic: () => onStartRecording(node.id),
              onStopMic: onStopRecording,
              isMicRecording: isRecording && recordingTargetNodeId === node.id,
              micStatus: micStatusByNodeId[node.id] ?? "Microphone ready",
            }
          : {}),
      },
    }));
  }, [
    edges,
    unbundle,
    editingLocked,
    isRecording,
    micStatusByNodeId,
    nodes,
    onStartRecording,
    onStopRecording,
    recordingTargetNodeId,
    removeNode,
    initIsaacGr00tForNode,
    onNodeDataChange,
    toggleRealSenseBridge,
    uploadAudioForNode,
    uploadDatasetForNode,
    uploadInitialSettingsForNode,
    uploadImageForNode,
    uploadVideoForNode,
  ]);

  const workflowDetails = useMemo(() => workflowView(renderedNodes, edges), [renderedNodes, edges]);
  const canvasView = useMemo(() => {
    const bundled = bundleView(renderedNodes, edges);
    const view = workflowView(bundled.nodes, bundled.edges);
    return { ...view, nodes: view.nodes.map((node) => node.type === "workflow_group" ? {
      ...node, data: { ...node.data, onShowDetails: () => { setBundleWindowId(node.id); setSelectedNode(null); } },
    } : node) };
  }, [renderedNodes, edges]);
  const openBundle = nodes.find((node) => node.id === bundleWindowId && node.type === "graph_bundle")
    || workflowDetails.nodes.find((node) => node.id === bundleWindowId && node.type === "workflow_group");
  const innerIds = new Set<string>(openBundle?.data.memberIds || []);
  const selectGraph = (id: string) => {
    setActiveGraphId(id); setSelectedNode(null);
    const visibleIds = new Set(canvasView.nodes.filter((node) => graphId(node) === id).map((node) => node.id));
    if (!editingLocked) setNodes((prev) => prev.map((node) => ({ ...node, selected: visibleIds.has(node.id) })));
    void flowInstanceRef.current?.fitView({ nodes: [...visibleIds].map((nodeId) => ({ id: nodeId })), duration: 250, padding: 0.15 });
  };

  const minimap = useMemo(
    () => (
      <MiniMap
        zoomable
        pannable
        nodeColor={() => "#1e293b"}
        maskColor="rgba(2,6,23,0.55)"
      />
    ),
    []
  );

  const previewNode = useMemo(
    () =>
      [...nodes].reverse().find((node) => {
        const data = node.data as Record<string, unknown> | undefined;
        return Boolean(data?.capturedFrame || data?.lastImageFrame);
      }) ?? null,
    [nodes]
  );
  const previewData = previewNode?.data as Record<string, unknown> | undefined;
  const previewImage = String(previewData?.capturedFrame || previewData?.lastImageFrame || "");
  const previewDepthImage = String(previewData?.lastDepthImageFrame || "");
  const previewLabel = String(previewData?.label || humanizeNodeType(previewNode?.type ?? ""));

  return (
    <div className="app-shell">
      <Sidebar onAddNode={onAddNode} graphs={graphs} activeGraphId={activeGraphId}
        onSelectGraph={selectGraph} disabled={editingLocked || Boolean(bundleWindowId)} />

      <main className="canvas-shell">
        <div className="topbar">
          <div className="topbar-group">
            <span className="topbar-group-label">GRAPH</span>
            <button className="secondary-btn" onClick={newGraph} disabled={editingLocked}>New Graph</button>
            <button className="secondary-btn" onClick={() => { setPendingTemplateId(graphTemplateId); setLoadOpen(true); }} disabled={editingLocked}>Load</button>
            <button className="secondary-btn" onClick={createBundle} disabled={editingLocked || !graphs.length || graphs.find((graph) => graph.id === activeGraphId)?.bundled}>Bundle Selected Graph</button>
          </div>
          <div className="topbar-group run-actions">
            <span className="topbar-group-label">RUN</span>
            <button className="primary-btn" onClick={onRun} title="Run all graphs on the canvas, including bundle contents" disabled={editingLocked || !nodes.length}>{isRunStarting ? "Starting..." : "Run Graph"}</button>
            <button className="secondary-btn" onClick={onCancelRun} disabled={!runtimeSessionId || !ACTIVE_RUNTIME_STATES.has(runtimeStatus)}>{isRunCancelling || runtimeStatus === "cancelling" ? "Cancelling..." : "Cancel Run"}</button>
            <button className="secondary-btn" onClick={saveGraph}>Save</button>
          </div>
        </div>

        <section className="graph-titlebar">
          <div>
            <span className="workspace-eyebrow">WORKFLOW DESIGNER</span>
            <h1>{graphs.find((graph) => graph.id === activeGraphId)?.label ?? "새 그래프"}</h1>
            <p>{canvasView.nodes.length}개 표시 단계 · 내부 {nodes.filter((node) => node.type !== "graph_bundle").length}개 노드 · {edges.length}개 연결</p>
          </div>
          <div className={`status-pill tone-${runOverview.state}`}>{toneLabel(runOverview.state)}</div>
        </section>

        {loadOpen && (
          <div className="graph-load-overlay">
            <div className="graph-load-dialog" onKeyDown={(event) => { if (event.key === "Escape") setLoadOpen(false); }} role="dialog" aria-modal="true" aria-labelledby="graph-load-title">
              <div className="graph-load-header">
                <div>
                  <div id="graph-load-title" className="graph-load-title">Load Graph</div>
                  <div className="graph-load-subtitle">그래프를 선택한 뒤 적용 방식을 고르세요.</div>
                </div>
                <button autoFocus className="secondary-btn" onClick={() => setLoadOpen(false)}>
                  Close
                </button>
              </div>
              <div className="graph-load-body">
                <div className="graph-template-list">
                  {GRAPH_TEMPLATE_OPTIONS.map((template) => {
                    const isSelected = pendingTemplateId === template.id;
                    return (
                      <button
                        key={template.id}
                        className={`graph-template-option${isSelected ? " selected" : ""}`}
                        onClick={() => setPendingTemplateId(template.id)}
                      >
                        <span className="graph-template-name">{template.label}</span>
                        <span className="graph-template-description">{template.description}</span>
                      </button>
                    );
                  })}
                </div>
                <div className="graph-load-preview">
                  <div className="graph-load-preview-label">Selected Graph</div>
                  <div className="graph-load-preview-title">
                    {GRAPH_TEMPLATES[pendingTemplateId].label}
                  </div>
                  <div className="graph-load-preview-description">
                    {GRAPH_TEMPLATES[pendingTemplateId].description}
                  </div>
                  <div className="graph-load-preview-stats">
                    <span>{GRAPH_TEMPLATES[pendingTemplateId].nodes.length} nodes</span>
                    <span>{GRAPH_TEMPLATES[pendingTemplateId].edges.length} edges</span>
                  </div>
                </div>
              </div>
              <div className="graph-load-actions">
                <button className="secondary-btn" disabled={editingLocked} onClick={() => graphFileRef.current?.click()}>JSON import</button>
                <button className="secondary-btn" disabled={editingLocked} onClick={() => applyGraphTemplate(pendingTemplateId)}>
                  덮어쓰기
                </button>
                <button className="primary-btn" disabled={editingLocked} onClick={addTemplate}>
                  추가하기
                </button>
              </div>
            </div>
          </div>
        )}

        <div className="flow-wrap">
          <ReactFlow key={flowKey} nodes={canvasView.nodes}
            onInit={(instance) => { flowInstanceRef.current = instance; }}
            edges={canvasView.edges}
            onNodesChange={editingLocked ? undefined : onNodesChange}
            onEdgesChange={editingLocked ? undefined : onEdgesChange}
            onConnect={editingLocked ? undefined : onConnect}
            nodesDraggable={!editingLocked}
            isValidConnection={isValidConnection}
            onNodeClick={(_, node) => { setActiveGraphId(graphId(node)); setSelectedNode(["graph_bundle", "workflow_group"].includes(node.type || "") ? null : node); }}
            nodeTypes={nodeTypes}
            nodesConnectable={!editingLocked}
            connectionRadius={32}
            fitView
          >
            <Background gap={24} size={1} color="#243a55" />
            <Controls />
            {minimap}
          </ReactFlow>
        </div>

        <input ref={graphFileRef} type="file" accept=".json,application/json" hidden onChange={(event) => {
          const file = event.target.files?.[0]; event.target.value = ""; if (file) void importGraph(file);
        }} />
        {openBundle && <div className="bundle-window-overlay">
          <section className="bundle-window" role="dialog" aria-modal="false" aria-labelledby="bundle-title" onKeyDown={(event) => {
            if (event.key === "Escape") { setBundleWindowId(null); setSelectedNode(null); }
          }}>
            <header className="bundle-window-header"><div><h2 id="bundle-title">{openBundle.data.label}</h2>
              <p>{innerIds.size} nodes · 노드를 클릭하면 오른쪽 Inspector에서 수정할 수 있습니다.</p></div>
              <button autoFocus className="secondary-btn" onClick={() => { setBundleWindowId(null); setSelectedNode(null); }}>Close</button></header>
            <div className="bundle-window-flow">
              <ReactFlow key={openBundle.id} nodes={renderedNodes.filter((node) => innerIds.has(node.id))}
                edges={edges.filter((edge) => innerIds.has(edge.source) && innerIds.has(edge.target))}
                onNodesChange={editingLocked ? undefined : onNodesChange} onEdgesChange={editingLocked ? undefined : onEdgesChange}
                onConnect={editingLocked ? undefined : onConnect} isValidConnection={isValidConnection}
                nodesDraggable={!editingLocked} nodesConnectable={!editingLocked}
                onNodeClick={(_, node) => setSelectedNode(node)} nodeTypes={nodeTypes} fitView>
                <Background gap={24} size={1} color="#64748b" /><Controls />
              </ReactFlow>
            </div>
          </section>
        </div>}

        <section className="workspace-dock">
          <div className="output-panel">
            <WorkflowReports nodes={nodes.filter((node) => graphId(node) === activeGraphId)} results={runtimeReports} />
            <section className={`run-strip tone-${runOverview.state}`}>
              <div className="run-hero">
                <div className="run-title-row">
                  <div className={`status-pill tone-${runOverview.state}`}>{toneLabel(runOverview.state)}</div>
                  <h2>{runOverview.title}</h2>
                </div>
                <p>{runOverview.detail}</p>
              </div>

              <div className="metric-grid">
                {runOverview.metrics.map((metric) => (
                  <div key={metric.label} className={`metric-card tone-${metric.tone ?? "idle"}`}>
                    <div className="metric-label">{metric.label}</div>
                    <div className="metric-value">{metric.value}</div>
                    <div className="metric-detail">{metric.detail}</div>
                  </div>
                ))}
              </div>
            </section>
            <section className="dock-story-panel">
              <div className="dock-panel-heading">
                <strong>실행 결과</strong>
                <span>{storyEntries.length} 단계</span>
              </div>
              <div className="dock-story-list">
                {storyEntries.length === 0 ? (
                  <div className="story-empty">그래프를 실행하면 단계별 결과가 여기에 표시됩니다.</div>
                ) : (
                  storyEntries.map((entry) => (
                    <article key={entry.id} className={`dock-story-entry tone-${entry.state}`}>
                      <span className="dock-story-dot" />
                      <div>
                        <strong>{entry.title}</strong>
                        <p>{entry.detail}</p>
                      </div>
                    </article>
                  ))
                )}
              </div>
            </section>
          </div>

          <section className="preview-panel">
            <div className="dock-panel-heading">
              <div>
                <strong>실시간 영상 / 시뮬레이션</strong>
                <span className="preview-source">{previewNode ? previewLabel : "센서 입력 대기 중"}</span>
              </div>
              <span className={`preview-live-indicator${previewImage ? " is-live" : ""}`}>
                {previewImage ? "연결됨" : "대기"}
              </span>
            </div>
            <div className="preview-image-wrap">
              {previewImage ? (
                <img src={previewImage} alt={`${previewLabel} 센서 프리뷰`} />
              ) : (
                <div className="preview-placeholder">
                  <span className="preview-placeholder-icon">◉</span>
                  <strong>센서 영상을 기다리는 중</strong>
                  <span>카메라 또는 원격 이미지 노드를 그래프에 추가하세요.</span>
                </div>
              )}
            </div>
            <div className="preview-side-list">
              <div className="preview-side-card">
                <span>Depth / RealSense</span>
                {previewDepthImage ? (
                  <img src={previewDepthImage} alt="RealSense depth 프리뷰" />
                ) : (
                  <div className="preview-side-empty">Depth 입력 대기</div>
                )}
              </div>
              <div className="preview-side-card preview-graph-stats">
                <span>그래프 상태</span>
                <strong>{nodes.length} Nodes</strong>
                <strong>{edges.length} Connections</strong>
              </div>
            </div>
          </section>
        </section>
      </main>

      <Inspector
        executionActive={isRunStarting || (runtimeSessionId !== null && ACTIVE_RUNTIME_STATES.has(runtimeStatus))}
        selectedNode={selectedNode}
        nodeCount={nodes.length}
        edgeCount={edges.length}
        logs={logs}
        onNodeDataChange={onNodeDataChange}
      />
    </div>
  );
}
