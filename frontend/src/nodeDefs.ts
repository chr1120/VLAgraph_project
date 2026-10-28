// src/nodeDefs.ts

export type PinType =
  | "EXEC"
  | "TEXT"
  | "IMAGE"
  | "JSON"
  | "DATASET"
  | "MODEL"
  | "ACTION"
  | "NUMBER"
  | "ANY"
  | "FILE";

export type PinDef = {
  id: string;
  label: string;
  pinType: PinType;
  kind: "target" | "source";
};

export type NodeDef = {
  type: string;
  label: string;
  category: string;
  width?: number;
  height?: number;
  inputs: PinDef[];
  outputs: PinDef[];
  defaults?: Record<string, any>;
};

const SENSOR_IMAGE_OUTPUTS: PinDef[] = [
  { id: "image", label: "Image", pinType: "IMAGE", kind: "source" },
  { id: "sensor", label: "Sensor", pinType: "JSON", kind: "source" },
];

const SENSOR_LIDAR_OUTPUTS: PinDef[] = [
  { id: "lidar", label: "LiDAR", pinType: "JSON", kind: "source" },
  { id: "sensor", label: "Sensor", pinType: "JSON", kind: "source" },
];

const SENSOR_FUSED_OUTPUTS: PinDef[] = [
  { id: "image", label: "Image", pinType: "IMAGE", kind: "source" },
  { id: "lidar", label: "LiDAR", pinType: "JSON", kind: "source" },
  { id: "sensor", label: "Sensor", pinType: "JSON", kind: "source" },
];

const REALSENSE_OUTPUTS: PinDef[] = [
  { id: "image", label: "RGB Image", pinType: "IMAGE", kind: "source" },
  { id: "depth_image", label: "Depth Image", pinType: "IMAGE", kind: "source" },
  { id: "depth", label: "Depth", pinType: "JSON", kind: "source" },
  { id: "sensor", label: "RGB-D Sensor", pinType: "JSON", kind: "source" },
];

const MEDIAPIPE_POSE_OUTPUTS: PinDef[] = [
  { id: "image", label: "Pose Image", pinType: "IMAGE", kind: "source" },
  { id: "pose", label: "Pose", pinType: "JSON", kind: "source" },
];

const RGBD_POSE_FUSION_OUTPUTS: PinDef[] = [
  { id: "image", label: "Fused Image", pinType: "IMAGE", kind: "source" },
  { id: "pose3d", label: "3D Pose", pinType: "JSON", kind: "source" },
  { id: "skeleton", label: "Skeleton", pinType: "JSON", kind: "source" },
  { id: "motion", label: "Motion", pinType: "JSON", kind: "source" },
  { id: "skill", label: "Skill", pinType: "TEXT", kind: "source" },
];

const SCENE_SCAN_OUTPUTS: PinDef[] = [
  { id: "scan", label: "Scan", pinType: "JSON", kind: "source" },
  { id: "objects", label: "Objects", pinType: "JSON", kind: "source" },
  { id: "robot", label: "Robot", pinType: "JSON", kind: "source" },
  { id: "scene_id", label: "Scene ID", pinType: "TEXT", kind: "source" },
];

const SCENE_CONTEXT_RESET_OUTPUTS: PinDef[] = [
  { id: "scene", label: "Scene", pinType: "JSON", kind: "source" },
  { id: "scene_id", label: "Scene ID", pinType: "TEXT", kind: "source" },
  { id: "reset", label: "Reset", pinType: "JSON", kind: "source" },
];

const SPATIAL_MEMORY_OUTPUTS: PinDef[] = [
  { id: "record", label: "Record", pinType: "JSON", kind: "source" },
  { id: "dataset", label: "Dataset", pinType: "DATASET", kind: "source" },
];

const SPATIAL_RETRIEVE_OUTPUTS: PinDef[] = [
  { id: "memories", label: "Memories", pinType: "JSON", kind: "source" },
  { id: "records", label: "Records", pinType: "JSON", kind: "source" },
];

const SPATIAL_CONTEXT_OUTPUTS: PinDef[] = [
  { id: "text", label: "Text", pinType: "TEXT", kind: "source" },
  { id: "context", label: "Context", pinType: "TEXT", kind: "source" },
  { id: "memories", label: "Memories", pinType: "JSON", kind: "source" },
];

const SENSOR_REMOTE_REQUEST_DEFAULTS = {
  remoteMethod: "GET",
  remoteHeaders: {},
  remoteBody: {},
  requestTimeoutSec: 10,
};

export const DEFAULT_UNITY_IMAGE_URL =
  "http://localhost:9000/api/unity/image?client_id=isaacsim01";

export const DEFAULT_UNITY_LIDAR_URL =
  "http://localhost:9000/api/unity/lidar?client_id=isaacsim01";

const SENSOR_CAMERA_DEFAULTS = {
  cameraDeviceId: "",
  cameraFacingMode: "environment",
  captureWidth: 1280,
  captureHeight: 720,
  capturedFrame: "",
  lastCapturedAt: "",
};

const SENSOR_REMOTE_IMAGE_DEFAULTS = {
  remoteImageUrl: "",
  remoteImageTransport: "snapshot",
  rtspTransport: "tcp",
  ffmpegPath: "ffmpeg",
};

const SENSOR_REMOTE_IMAGE_NODE_DEFAULTS = {
  ...SENSOR_REMOTE_IMAGE_DEFAULTS,
  remoteImageUrl: DEFAULT_UNITY_IMAGE_URL,
};

const SENSOR_REMOTE_LIDAR_DEFAULTS = {
  remoteLidarUrl: DEFAULT_UNITY_LIDAR_URL,
  lidarFormat: "auto",
  lidarObstacleDistance: 1.0,
  lidarSample: {},
};

const REALSENSE_D435I_DEFAULTS = {
  realsenseSerial: "",
  remoteRgbdUrl: "http://host.docker.internal:9180/frame",
  captureWidth: 640,
  captureHeight: 480,
  fps: 30,
  frameTimeoutMs: 5000,
  warmupFrames: 4,
  depthMinMeters: 0.2,
  depthMaxMeters: 4.0,
  requestTimeoutSec: 10,
  mockWhenUnavailable: true,
  realsensePreviewEnabled: false,
  realsenseBridgeStatus: "off",
  realsenseBridgeMessage: "",
};

const MEDIAPIPE_POSE_DEFAULTS = {
  modelComplexity: 1,
  minDetectionConfidence: 0.5,
  minTrackingConfidence: 0.5,
  minPoseVisibility: 0.35,
};

const RGBD_POSE_FUSION_DEFAULTS = {
  depthSampleRadius: 3,
  minPoseVisibility: 0.35,
  landmarkSmoothing: 0.0,
  landmarkMaxDelta: 0.0,
  maxMotionFrames: 40,
  motionLabel: "",
  prompt: "",
};

export const FIXED_SENSOR_SOURCE_MODE_BY_TYPE: Record<string, string> = {
  sensor_camera: "camera",
  sensor_remote_image: "remote_image",
  sensor_remote_lidar: "remote_lidar",
  sensor_fused: "fused",
};

export function isSensorNodeType(type: string | null | undefined) {
  const normalized = String(type ?? "").trim();
  return normalized === "sensor_input" || normalized in FIXED_SENSOR_SOURCE_MODE_BY_TYPE;
}

export function getSensorSourceMode(
  nodeType: string | null | undefined,
  sourceMode: unknown
) {
  const normalizedType = String(nodeType ?? "").trim();
  if (normalizedType in FIXED_SENSOR_SOURCE_MODE_BY_TYPE) {
    return FIXED_SENSOR_SOURCE_MODE_BY_TYPE[normalizedType];
  }

  const normalizedMode = String(sourceMode ?? "camera").trim().toLowerCase();
  return normalizedMode || "camera";
}

export function resolveSensorRemoteImageUrl(
  nodeType: string | null | undefined,
  data: Record<string, unknown> | null | undefined
) {
  const configuredUrl = String(data?.remoteImageUrl ?? "").trim();
  if (configuredUrl) {
    return configuredUrl;
  }

  const sourceMode = getSensorSourceMode(nodeType, data?.sourceMode);
  if (sourceMode === "remote_image") {
    return DEFAULT_UNITY_IMAGE_URL;
  }

  return "";
}

export const NODE_DEF_BY_TYPE: Record<string, NodeDef> = {
  robot_setup: {
    type: "robot_setup", label: "로봇 설정", category: "Spatial", width: 360,
    inputs: [],
    outputs: [
      { id: "robot_config", label: "Robot Configuration", pinType: "JSON", kind: "source" },
      { id: "profile", label: "Robot Profile", pinType: "JSON", kind: "source" },
      { id: "summary", label: "Setup Summary", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "로봇 설정", robotModel: "fanuc_m710ic20l", settingsMode: "preset", instanceId: "fanuc_01",
      omniverseEndpoint: "ws://172.16.110.116:9910/omniverse",
      robotPrimPath: "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/HFS1_7Axis_LH/lnk3/FANUC_M710/Robot",
      memoryPath: "spatial_memory/spatial_memory.jsonl", usdPath: "", customProfileId: "", customModelName: "",
      toolOffsetM: [], manualProfile: {}, prepareSettings: {} },
  },
  prompt: {
    type: "prompt",
    label: "Prompt Node",
    category: "Prompt",
    width: 260,
    inputs: [],
    outputs: [{ id: "text", label: "Text", pinType: "TEXT", kind: "source" }],
    defaults: {
      label: "Prompt Node",
      text: "",
    },
  },

  image_input: {
    type: "image_input",
    label: "Image Input Node",
    category: "Input",
    width: 260,
    inputs: [],
    outputs: [{ id: "image", label: "Image", pinType: "IMAGE", kind: "source" }],
    defaults: {
      label: "Image Input Node",
      imageUrl: "",
    },
  },

  sensor_input: {
    type: "sensor_input",
    label: "Sensor Input Node",
    category: "Sensor",
    width: 320,
    inputs: [],
    outputs: SENSOR_FUSED_OUTPUTS,
    defaults: {
      label: "Sensor Input Node",
      sourceMode: "camera",
      ...SENSOR_CAMERA_DEFAULTS,
      ...SENSOR_REMOTE_REQUEST_DEFAULTS,
      ...SENSOR_REMOTE_IMAGE_DEFAULTS,
      ...SENSOR_REMOTE_LIDAR_DEFAULTS,
    },
  },

  sensor_camera: {
    type: "sensor_camera",
    label: "Camera Sensor Node",
    category: "Sensor",
    width: 320,
    inputs: [],
    outputs: SENSOR_IMAGE_OUTPUTS,
    defaults: {
      label: "Camera Sensor Node",
      ...SENSOR_CAMERA_DEFAULTS,
    },
  },

  sensor_remote_image: {
    type: "sensor_remote_image",
    label: "Remote Image Sensor Node",
    category: "Sensor",
    width: 320,
    inputs: [
      { id: "remoteImageUrl", label: "Image URL", pinType: "TEXT", kind: "target" },
    ],
    outputs: SENSOR_IMAGE_OUTPUTS,
    defaults: {
      label: "Remote Image Sensor Node",
      ...SENSOR_REMOTE_REQUEST_DEFAULTS,
      ...SENSOR_REMOTE_IMAGE_NODE_DEFAULTS,
    },
  },

  sensor_remote_lidar: {
    type: "sensor_remote_lidar",
    label: "Remote LiDAR Sensor Node",
    category: "Sensor",
    width: 320,
    inputs: [
      { id: "remoteLidarUrl", label: "LiDAR URL", pinType: "TEXT", kind: "target" },
    ],
    outputs: SENSOR_LIDAR_OUTPUTS,
    defaults: {
      label: "Remote LiDAR Sensor Node",
      ...SENSOR_REMOTE_REQUEST_DEFAULTS,
      ...SENSOR_REMOTE_LIDAR_DEFAULTS,
    },
  },

  sensor_fused: {
    type: "sensor_fused",
    label: "Fused Sensor Node",
    category: "Sensor",
    width: 320,
    inputs: [],
    outputs: SENSOR_FUSED_OUTPUTS,
    defaults: {
      label: "Fused Sensor Node",
      ...SENSOR_CAMERA_DEFAULTS,
      ...SENSOR_REMOTE_REQUEST_DEFAULTS,
      ...SENSOR_REMOTE_IMAGE_DEFAULTS,
      ...SENSOR_REMOTE_LIDAR_DEFAULTS,
    },
  },

  realsense_d435i_sensor: {
    type: "realsense_d435i_sensor",
    label: "RealSense D435i Sensor",
    category: "RGB-D",
    width: 340,
    inputs: [],
    outputs: REALSENSE_OUTPUTS,
    defaults: {
      label: "RealSense D435i Sensor",
      ...REALSENSE_D435I_DEFAULTS,
    },
  },

  mediapipe_pose: {
    type: "mediapipe_pose",
    label: "MediaPipe Pose",
    category: "RGB-D",
    width: 320,
    inputs: [
      { id: "image", label: "Image", pinType: "IMAGE", kind: "target" },
    ],
    outputs: MEDIAPIPE_POSE_OUTPUTS,
    defaults: {
      label: "MediaPipe Pose",
      ...MEDIAPIPE_POSE_DEFAULTS,
    },
  },

  rgbd_pose_fusion: {
    type: "rgbd_pose_fusion",
    label: "RGB-D Pose Fusion",
    category: "RGB-D",
    width: 340,
    inputs: [
      { id: "image", label: "Image", pinType: "IMAGE", kind: "target" },
      { id: "depth", label: "Depth", pinType: "JSON", kind: "target" },
      { id: "pose", label: "Pose", pinType: "JSON", kind: "target" },
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
    ],
    outputs: RGBD_POSE_FUSION_OUTPUTS,
    defaults: {
      label: "RGB-D Pose Fusion",
      ...RGBD_POSE_FUSION_DEFAULTS,
      landmarkSmoothing: 0.58,
      landmarkMaxDelta: 0.22,
    },
  },

  video_motion_input: {
    type: "video_motion_input",
    label: "Video Motion Input",
    category: "Motion",
    width: 300,
    inputs: [],
    outputs: [
      { id: "video", label: "Video", pinType: "FILE", kind: "source" },
      { id: "label", label: "Label", pinType: "TEXT", kind: "source" },
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "source" },
    ],
    defaults: {
      label: "Video Motion Input",
      videoPath: "",
      motionLabel: "walk_forward",
      prompt: "",
    },
  },

  motion_extract: {
    type: "motion_extract",
    label: "Motion Extract Node",
    category: "Motion",
    width: 300,
    inputs: [
      { id: "video", label: "Video", pinType: "FILE", kind: "target" },
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "label", label: "Label", pinType: "TEXT", kind: "target" },
    ],
    outputs: [
      { id: "motion", label: "Motion", pinType: "JSON", kind: "source" },
      { id: "skill", label: "Skill", pinType: "TEXT", kind: "source" },
      { id: "dataset", label: "Trace", pinType: "DATASET", kind: "source" },
    ],
    defaults: {
      label: "Motion Extract Node",
      videoPath: "",
      motionLabel: "",
      prompt: "",
      everyNFrames: 2,
      maxFrames: 0,
      minVisibility: 0.35,
      includeFrames: false,
      outputName: "",
    },
  },

  motion_retarget: {
    type: "motion_retarget",
    label: "Motion Retarget Node",
    category: "Motion",
    width: 300,
    inputs: [
      { id: "motion", label: "Motion", pinType: "JSON", kind: "target" },
      { id: "skill", label: "Skill", pinType: "TEXT", kind: "target" },
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
    ],
    outputs: [
      { id: "action", label: "Action", pinType: "ACTION", kind: "source" },
      { id: "skill", label: "Skill", pinType: "TEXT", kind: "source" },
    ],
    defaults: {
      label: "Motion Retarget Node",
      motionLabel: "",
      prompt: "",
      speed: 0.45,
      stepScale: 0.25,
      turnScale: 0.2,
      durationSec: 2.0,
      fullBodyRetarget: true,
      targetEmbodiment: "",
      emitG1JointTargets: false,
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
      jointSmoothing: 0.25,
      jointMaxDeltaRad: 0.10,
      jointAverageWindow: 5,
      maxPoseFrames: 40,
      poseScale: 1.0,
      poseCalibration: "auto",
      lowerBodyMode: "auto",
      inactiveArmDeadzone: 0.14,
      limbMinVisibility: 0.42,
    },
  },

  scene_scan: {
    type: "scene_scan",
    label: "Scene Scan",
    category: "Spatial",
    width: 320,
    inputs: [
      { id: "state", label: "State", pinType: "JSON", kind: "target" },
    ],
    outputs: SCENE_SCAN_OUTPUTS,
    defaults: {
      label: "Scene Scan",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniversePrimPath: "/World/GROOT_Humanoid",
      sceneRootPath: "/World",
      maxObjects: 256,
      timeoutSec: 15,
    },
  },

  fanuc_usd_geometry_read: {
    type: "fanuc_usd_geometry_read", label: "FANUC USD Geometry", category: "Spatial", width: 360,
    inputs: [],
    outputs: [{ id: "inventory", label: "USD Collision Inventory", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "FANUC USD Geometry", source: "bridge", usdPath: "",
      omniverseEndpoint: "ws://host.docker.internal:9910/omniverse",
      robotPrimPath: "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/HFS1_7Axis_LH/lnk3/FANUC_M710/Robot",
      rootPath: "/World", maxColliders: 4096, timeoutSec: 15,
    },
  },

  fanuc_usd_proxy_build: {
    type: "fanuc_usd_proxy_build", label: "FANUC USD Proxy", category: "Spatial", width: 360,
    inputs: [
      { id: "inventory", label: "USD Collision Inventory", pinType: "JSON", kind: "target" },
      { id: "review", label: "Geometry Review (optional)", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "scene", label: "Robot Proxy Scene", pinType: "JSON", kind: "source" },
      { id: "coverage_report", label: "Geometry Coverage", pinType: "JSON", kind: "source" },
      { id: "review_plan", label: "Geometry Review Plan", pinType: "JSON", kind: "source" },
      { id: "binding_report", label: "USD / FK Binding", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "FANUC USD Proxy", paddingM: 0, excludedSelfPairs: [], geometryReview: {} },
  },

  fanuc_collision_check: {
    type: "fanuc_collision_check", label: "FANUC Geometry Collision", category: "Spatial", width: 360,
    inputs: [
      { id: "action", label: "Prepared Action", pinType: "JSON", kind: "target" },
      { id: "scene", label: "Collision Scene", pinType: "JSON", kind: "target" },
      { id: "inventory", label: "USD Inventory (optional)", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Initial Snapshot", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "collision_report", label: "Collision Report", pinType: "JSON", kind: "source" }],
    defaults: { label: "FANUC Geometry Collision", enabled: false, scene: {}, maxStepSec: 0.05, marginM: 0.01, maxSamples: 10000 },
  },

  fanuc_trajectory_validate: {
    type: "fanuc_trajectory_validate", label: "FANUC Trajectory Preflight", category: "Spatial", width: 360,
    inputs: [
      { id: "action", label: "Prepared Action", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Initial Snapshot", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "report", label: "Validation Report", pinType: "JSON", kind: "source" },
      { id: "segment_report", label: "LS Segment Review", pinType: "JSON", kind: "source" },
      { id: "preview_action", label: "Preview Only", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "FANUC Trajectory Preflight", maxVelocity: {}, maxAcceleration: {} },
  },

  fanuc_snapshot_read: {
    type: "fanuc_snapshot_read", label: "FANUC Physics Snapshot", category: "Spatial", width: 340,
    inputs: [],
    outputs: [{ id: "snapshot", label: "Snapshot", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "FANUC Physics Snapshot", omniverseEndpoint: "ws://host.docker.internal:9910/omniverse",
      robotPrimPath: "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/HFS1_7Axis_LH/lnk3/FANUC_M710/Robot",
      timeoutSec: 5,
    },
  },

  fanuc_world_context: {
    type: "fanuc_world_context", label: "FANUC Collection Context", category: "Spatial", width: 360,
    inputs: [
      { id: "scan", label: "Current Scene Scan", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Physics Snapshot", pinType: "JSON", kind: "target" },
      { id: "inventory", label: "Collision Inventory", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "context_report", label: "Collection Reconciliation", pinType: "JSON", kind: "source" },
      { id: "scene", label: "Reconciled Collision Scene", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "FANUC Collection Context", memoryPath: "spatial_memory/spatial_memory.jsonl",
      maxMemoryAgeSec: 86400, maxLiveAgeSec: 5, positionToleranceM: 0.02 },
  },
  fanuc_world_prediction: {
    type: "fanuc_world_prediction", label: "FANUC Prediction Handoff", category: "Spatial", width: 360,
    inputs: [
      { id: "action", label: "Prepared Action", pinType: "JSON", kind: "target" },
      { id: "context_report", label: "World Context", pinType: "JSON", kind: "target" },
      { id: "collision_report", label: "Geometry Sweep", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Physics Snapshot", pinType: "JSON", kind: "target" },
      { id: "scene", label: "Collision Scene", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "prediction_report", label: "Prediction Availability", pinType: "JSON", kind: "source" }],
    defaults: { label: "FANUC Prediction Handoff" },
  },
  execution_guard: {
    type: "execution_guard", label: "Execution Guard", category: "Spatial", width: 360,
    inputs: [
      { id: "action", label: "Prepared Action", pinType: "JSON", kind: "target" },
      { id: "report", label: "Validation Report", pinType: "JSON", kind: "target" },
      { id: "collision_report", label: "Collision Report", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Initial Snapshot", pinType: "JSON", kind: "target" },
      { id: "context_report", label: "Collection Reconciliation", pinType: "JSON", kind: "target" },
      { id: "prediction_report", label: "Prediction Report", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "action", label: "Guarded Action", pinType: "JSON", kind: "source" },
      { id: "decision", label: "Execution Decision", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "Execution Guard", validationPolicy: "full", requireInitialSnapshot: false, maxSnapshotAgeSec: 5 },
  },

  execution_observe: {
    type: "execution_observe", label: "Execution Observe", category: "Spatial", width: 340,
    inputs: [
      { id: "command_id", label: "Command ID", pinType: "TEXT", kind: "target" },
      { id: "result", label: "Execution Result", pinType: "JSON", kind: "target" },
      { id: "robot_config", label: "Robot Config", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "execution", label: "Execution", pinType: "JSON", kind: "source" },
      { id: "completed", label: "Completed", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "Execution Observe", omniverseEndpoint: "ws://host.docker.internal:9910/omniverse",
      robotPrimPath: "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/HFS1_7Axis_LH/lnk3/FANUC_M710/Robot",
      commandId: "", timeoutSec: 5,
      waitForCompletion: false, completionTimeoutSec: 120, pollIntervalSec: .25,
    },
  },

  fanuc_state_read: {
    type: "fanuc_state_read",
    label: "FANUC State Read",
    category: "Spatial",
    width: 340,
    inputs: [],
    outputs: [
      { id: "state", label: "Arm Observation", pinType: "JSON", kind: "source" },
      { id: "raw_state", label: "Raw Bridge State", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "FANUC State Read",
      omniverseEndpoint: "ws://host.docker.internal:9910/omniverse",
      robotPrimPath: "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/HFS1_7Axis_LH/lnk3/FANUC_M710/Robot",
      timeoutSec: 5,
    },
  },

  world_state_build: {
    type: "world_state_build",
    label: "World State Build",
    category: "Spatial",
    width: 340,
    inputs: [
      { id: "scan", label: "Scan", pinType: "JSON", kind: "target" },
      { id: "episode_id", label: "Episode ID", pinType: "TEXT", kind: "target" },
      { id: "scene_version", label: "Scene Version", pinType: "TEXT", kind: "target" },
    ],
    outputs: [
      { id: "world_state", label: "World State", pinType: "JSON", kind: "source" },
      { id: "prediction_ready", label: "Prediction Ready", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "World State Build", episodeId: "", sceneVersion: "" },
  },

  robot_observations_read: {
    type: "robot_observations_read", label: "Robot Observations Read", category: "Spatial", width: 340,
    inputs: [],
    outputs: [
      { id: "world_state", label: "World State", pinType: "JSON", kind: "source" },
      { id: "navigation_map", label: "Navigation Map", pinType: "JSON", kind: "source" },
      { id: "navigation_maps", label: "Maps by Robot", pinType: "JSON", kind: "source" },
      { id: "scan", label: "Scan", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "Robot Observations Read", omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      robotPrimPath: "/World/AMR", sceneVersion: "v1", timeoutSec: 10 },
  },

  multi_robot_context: {
    type: "multi_robot_context", label: "Multi Robot Context", category: "Spatial", width: 340,
    inputs: [
      { id: "world_state", label: "Shared World State", pinType: "JSON", kind: "target" },
      { id: "navigation_maps", label: "Maps by Robot", pinType: "JSON", kind: "target" },
      { id: "sensors", label: "Sensors by Robot", pinType: "JSON", kind: "target" },
      { id: "bindings", label: "Model Bindings", pinType: "JSON", kind: "target" },
      { id: "tasks", label: "Robot Tasks", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "batch", label: "Inference Batch", pinType: "JSON", kind: "source" }],
    defaults: { label: "Multi Robot Context", robotBindings: [], robotTasks: [] },
  },

  multi_robot_infer: {
    type: "multi_robot_infer", label: "Multi Robot Infer", category: "Spatial", width: 320,
    inputs: [{ id: "batch", label: "Inference Batch", pinType: "JSON", kind: "target" }],
    outputs: [{ id: "candidates", label: "Robot Candidates", pinType: "JSON", kind: "source" }],
    defaults: { label: "Multi Robot Infer", maxWorkers: 4 },
  },

  robot_candidate_select: {
    type: "robot_candidate_select", label: "Robot Candidate Select", category: "Spatial", width: 340,
    inputs: [
      { id: "candidates", label: "Robot Candidates", pinType: "JSON", kind: "target" },
      { id: "world_state", label: "Shared World State", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "candidate", label: "Candidate for Preparation", pinType: "JSON", kind: "source" },
      { id: "action", label: "Policy Preview", pinType: "ACTION", kind: "source" },
      { id: "plan", label: "AMR Plan", pinType: "JSON", kind: "source" },
      { id: "world_state", label: "Selected Robot World", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "Robot Candidate Select", robotPrimPath: "", requireBatchSuccess: true },
  },

  fanuc_policy_prepare: {
    type: "fanuc_policy_prepare", label: "FANUC Policy Prepare", category: "Spatial", width: 340,
    inputs: [
      { id: "candidate", label: "Policy Candidate", pinType: "JSON", kind: "target" },
      { id: "world_state", label: "Selected Robot World", pinType: "JSON", kind: "target" },
      { id: "robot_config", label: "Robot Config", pinType: "JSON", kind: "target" },
      { id: "snapshot", label: "Initial Snapshot", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "action", label: "Prepared Action", pinType: "ACTION", kind: "source" },
      { id: "report", label: "Preflight Report", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "FANUC Policy Prepare", enableExecution: false },
  },

  amr_path_plan: {
    type: "amr_path_plan", label: "AMR Path Plan", category: "Spatial", width: 320,
    inputs: [
      { id: "world_state", label: "World State", pinType: "JSON", kind: "target" },
      { id: "navigation_map", label: "Navigation Map", pinType: "JSON", kind: "target" },
      { id: "goal", label: "Goal XY (m)", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "plan", label: "Plan", pinType: "JSON", kind: "source" }],
    defaults: { label: "AMR Path Plan", goalM: [3, 0], marginM: .1 },
  },

  amr_navigate: {
    type: "amr_navigate", label: "AMR Navigate (Simulation)", category: "Spatial", width: 340,
    inputs: [{ id: "plan", label: "Plan", pinType: "JSON", kind: "target" }],
    outputs: [
      { id: "execution", label: "Execution", pinType: "JSON", kind: "source" },
      { id: "command_id", label: "Command ID", pinType: "TEXT", kind: "source" },
    ],
    defaults: { label: "AMR Navigate (Simulation)", omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      enableExecution: false, commandId: "", navigationTimeoutSec: 120, timeoutSec: 10 },
  },

  amr_status: {
    type: "amr_status", label: "AMR Status", category: "Spatial", width: 300,
    inputs: [{ id: "command_id", label: "Command ID", pinType: "TEXT", kind: "target" }],
    outputs: [
      { id: "execution", label: "Execution", pinType: "JSON", kind: "source" },
      { id: "completed", label: "Completed", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "AMR Status", omniverseEndpoint: "ws://host.docker.internal:9900/omniverse", commandId: "", timeoutSec: 5 },
  },

  amr_stop: {
    type: "amr_stop", label: "AMR Stop", category: "Spatial", width: 300,
    inputs: [{ id: "command_id", label: "Command ID", pinType: "TEXT", kind: "target" }],
    outputs: [{ id: "execution", label: "Execution", pinType: "JSON", kind: "source" }],
    defaults: { label: "AMR Stop", omniverseEndpoint: "ws://host.docker.internal:9900/omniverse", commandId: "", timeoutSec: 5 },
  },

  scene_context_reset: {
    type: "scene_context_reset",
    label: "Scene Context Reset",
    category: "Spatial",
    width: 320,
    inputs: [
      { id: "scan", label: "Scan", pinType: "JSON", kind: "target" },
      { id: "state", label: "State", pinType: "JSON", kind: "target" },
    ],
    outputs: SCENE_CONTEXT_RESET_OUTPUTS,
    defaults: {
      label: "Scene Context Reset",
      clearSceneMemory: false,
      memoryPath: "spatial_memory/spatial_memory.jsonl",
    },
  },

  spatial_memory_save: {
    type: "spatial_memory_save",
    label: "Spatial Memory Save",
    category: "Spatial",
    width: 340,
    inputs: [
      { id: "scan", label: "Scan", pinType: "JSON", kind: "target" },
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "action", label: "Action", pinType: "ACTION", kind: "target" },
      { id: "result", label: "Result", pinType: "JSON", kind: "target" },
    ],
    outputs: SPATIAL_MEMORY_OUTPUTS,
    defaults: {
      label: "Spatial Memory Save",
      memoryPath: "spatial_memory/spatial_memory.jsonl",
    },
  },

  spatial_rag_retrieve: {
    type: "spatial_rag_retrieve",
    label: "Spatial RAG Retrieve",
    category: "Spatial",
    width: 340,
    inputs: [
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "scan", label: "Scan", pinType: "JSON", kind: "target" },
      { id: "scene_id", label: "Scene ID", pinType: "TEXT", kind: "target" },
    ],
    outputs: SPATIAL_RETRIEVE_OUTPUTS,
    defaults: {
      label: "Spatial RAG Retrieve",
      memoryPath: "spatial_memory/spatial_memory.jsonl",
      topK: 5,
    },
  },

  spatial_context_builder: {
    type: "spatial_context_builder",
    label: "Spatial Context Builder",
    category: "Spatial",
    width: 340,
    inputs: [
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "memories", label: "Memories", pinType: "JSON", kind: "target" },
    ],
    outputs: SPATIAL_CONTEXT_OUTPUTS,
    defaults: {
      label: "Spatial Context Builder",
      maxLines: 5,
    },
  },

  robot_state: {
    type: "robot_state",
    label: "Robot State Node",
    category: "State",
    width: 260,
    inputs: [],
    outputs: [{ id: "state", label: "State", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "Robot State Node",
      robotState: {},
    },
  },

  unity_state: {
    type: "unity_state",
    label: "Robot State (Isaac Sim)",
    category: "Robot / Omniverse",
    width: 260,
    inputs: [],
    outputs: [{ id: "state", label: "State", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "Robot State (Isaac Sim)",
      clientId: "isaacsim01",
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      initTimeoutSec: 15,
    },
  },

  openvla_infer: {
    type: "openvla_infer",
    label: "OpenVLA Infer Node",
    category: "VLA",
    width: 280,
    inputs: [
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "image", label: "Image", pinType: "IMAGE", kind: "target" },
      { id: "state", label: "State", pinType: "JSON", kind: "target" },
      { id: "lidar", label: "LiDAR", pinType: "JSON", kind: "target" },
      { id: "sensor", label: "Sensor", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "action", label: "Action", pinType: "ACTION", kind: "source" }],
    defaults: {
      label: "OpenVLA Infer Node",
      modelName: "openvla-stub",
      actionFormat: "",
      unityCommand: "",
      waypointSpeed: 1.5,
    },
  },

  gr00t_infer: {
    type: "gr00t_infer",
    label: "NVIDIA GR00T Infer Node",
    category: "VLA",
    width: 300,
    inputs: [
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "image", label: "Image", pinType: "IMAGE", kind: "target" },
      { id: "state", label: "State", pinType: "JSON", kind: "target" },
      { id: "lidar", label: "LiDAR", pinType: "JSON", kind: "target" },
      { id: "sensor", label: "Sensor", pinType: "JSON", kind: "target" },
    ],
    outputs: [{ id: "action", label: "Action", pinType: "ACTION", kind: "source" }],
    defaults: {
      label: "NVIDIA GR00T Infer Node",
      modelName: "nvidia/GR00T-N1.7-3B",
      endpoint: "",
      embodimentTag: "REAL_G1",
      inferenceMode: "pytorch",
      actionHorizon: 40,
      actionFormat: "humanoid_step",
      unityCommand: "apply_humanoid_step",
      waypointSpeed: 1.0,
      requireImage: true,
    },
  },

  action_decode: {
    type: "action_decode",
    label: "Action Decode Node",
    category: "Postprocess",
    width: 280,
    inputs: [{ id: "action", label: "Action", pinType: "ACTION", kind: "target" }],
    outputs: [{ id: "decoded", label: "Decoded", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "Action Decode Node",
      decodeMode: "default",
    },
  },

  g1_skill_decode: {
    type: "g1_skill_decode",
    label: "G1 Skill Decode Node",
    category: "Postprocess",
    width: 300,
    inputs: [
      { id: "prompt", label: "Prompt", pinType: "TEXT", kind: "target" },
      { id: "action", label: "Action", pinType: "ACTION", kind: "target" },
      { id: "state", label: "State", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "action", label: "Skill Action", pinType: "ACTION", kind: "source" },
      { id: "skill", label: "Skill", pinType: "TEXT", kind: "source" },
      { id: "intent", label: "Intent", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "G1 Skill Decode Node",
      prompt: "",
      defaultSpeed: 0.55,
      durationSec: 3.0,
    },
  },

  safety_clamp: {
    type: "safety_clamp",
    label: "Safety Clamp Node",
    category: "Safety",
    width: 280,
    inputs: [{ id: "action", label: "Action", pinType: "JSON", kind: "target" }],
    outputs: [{ id: "safe_action", label: "Safe Action", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "Safety Clamp Node",
      enabled: true,
      maxDelta: 0.05,
    },
  },

  robot_execute: {
    type: "robot_execute",
    label: "Robot Execute Node",
    category: "Robot",
    width: 300,
    inputs: [{ id: "action", label: "Action", pinType: "JSON", kind: "target" }],
    outputs: [{ id: "result", label: "Result", pinType: "JSON", kind: "source" }],
    defaults: {
      label: "Robot Execute Node",
      executor: "robot-executor",
      commandId: "",
      mode: "omniverse", // simulate | unity | omniverse | ros

      // simulate
      simulateName: "default-sim",
      simulateSpeed: 1.0,
      dryRun: true,

      // shared robot metadata
      robotType: "humanoid",
      controllerMode: "locomotion",
      clientId: "isaacsim01",

      // unity
      unityEndpoint: "ws://host.docker.internal:9081/robot",
      unityScene: "VALConnect",
      unityObject: "GROOT_Humanoid",
      unityRobotType: "humanoid",
      unityControllerMode: "locomotion",
      unityCommand: "apply_humanoid_step",
      unityTimeoutSec: 15,
      unityStateClientId: "isaacsim01",
      stopRuntimeOnTerminate: false,

      // omniverse
      omniverseEndpoint: "ws://host.docker.internal:9900/omniverse",
      omniverseUrl: "omniverse://localhost/Projects/Robot",
      omniverseStage: "/World",
      omniversePrimPath: "/World/GROOT_Humanoid",
      omniverseCommand: "execute_policy",
      omniverseTimeoutSec: 15,
      omniverseWaitForResponse: true,

      // ros
      rosBridgeUrl: "ws://localhost:9090",
      rosTopic: "/robot/action_cmd",
      rosRobotName: "robot1",
      rosFrameId: "base_link",
      rosMsgType: "geometry_msgs/Twist",
    },
  },

  omy3m_joint_action: {
    type: "omy3m_joint_action",
    label: "OMY3M 6축 관절 Action",
    category: "Robot",
    width: 320,
    inputs: [
      { id: "prompt", label: "Instruction", pinType: "TEXT", kind: "target" },
      { id: "joint_positions", label: "Joint Positions", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "action", label: "OMY3M Action", pinType: "ACTION", kind: "source" },
      { id: "joint_target_map", label: "Joint Map", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "OMY3M 6축 관절 Action",
      jointNames: "joint1,joint2,joint3,joint4,joint5,joint6",
      jointPositions: "0,-0.5,0.8,0,0.4,0",
      gripperPosition: 0,
      jointLimitRad: 3.14159,
      durationSec: 1,
    },
  },

  omy3m_isaaclab_record: {
    type: "omy3m_isaaclab_record",
    label: "OMY3M IsaacLab 데이터 저장",
    category: "Dataset",
    width: 340,
    inputs: [
      { id: "prompt", label: "Instruction", pinType: "TEXT", kind: "target" },
      { id: "image", label: "Wrist RGB", pinType: "IMAGE", kind: "target" },
      { id: "state", label: "Robot State", pinType: "JSON", kind: "target" },
      { id: "action", label: "Commanded Action", pinType: "ACTION", kind: "target" },
      { id: "result", label: "Execution Result", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "dataset", label: "IsaacLab Dataset", pinType: "DATASET", kind: "source" },
      { id: "record", label: "Saved Step", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "OMY3M IsaacLab 데이터 저장",
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

  fanuc_ls_parse: {
    type: "fanuc_ls_parse",
    label: "FANUC LS 파서",
    category: "Robot",
    width: 320,
    inputs: [
      { id: "dataset", label: "FANUC LS File", pinType: "DATASET", kind: "target" },
      { id: "initialSettings", label: "Initial LS", pinType: "DATASET", kind: "target" },
    ],
    outputs: [
      { id: "program", label: "Parsed Program", pinType: "JSON", kind: "source" },
      { id: "trajectory", label: "FANUC Trajectory", pinType: "JSON", kind: "source" },
      { id: "summary", label: "Parse Summary", pinType: "JSON", kind: "source" },
    ],
    defaults: { label: "FANUC LS 파서" },
  },

  fanuc_m710_prepare: {
    type: "fanuc_m710_prepare",
    label: "FANUC M-710 실행 명령",
    category: "Robot",
    width: 340,
    inputs: [{ id: "program", label: "FANUC Program", pinType: "JSON", kind: "target" }],
    outputs: [
      { id: "action", label: "M-710 Trajectory", pinType: "ACTION", kind: "source" },
      { id: "preview", label: "Joint Preview", pinType: "JSON", kind: "source" },
      { id: "summary", label: "Safety Summary", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "FANUC M-710 J1~J6 실행 명령",
      totalDurationSec: 30,
      secondsPerFrame: 3,
      timingMode: "source_speed",
      jointPercentReferenceSec: 2,
      trajectoryTimeScale: 1.0,
      previewOnly: true,
      enableExecution: false,
      solveCartesianIk: true,
      requireAllIk: false,
      cartesianPositionScale: 1.0,
      cartesianAxisSigns: "1,1,1",
      jointAxisSigns: "1,1,1,1,1,1",
      initialJointAxisSigns: "1,1,1,1,1,1",
      externalAxisVectors: { E1: [-1, 0, 0] },
      externalAxisJointNames: { E1: "externalAxisJoint" },
      externalAxisRootMotionEnabled: true,
      externalAxisAffectsIk: false,
      cartesianOrientationScale: 1.0,
      uframeTransforms: {},
      utoolTransforms: {},
      ikOrientationWeight: 0.75,
      ikMaxIterations: 220,
      ikPositionToleranceM: 0.001,
      ikOrientationToleranceDeg: 1.0,
      ikMaxJointDeltaDeg: 140.0,
    },
  },

  fanuc_omy3m_retarget: {
    type: "fanuc_omy3m_retarget",
    label: "FANUC → OMY3M 리타게팅",
    category: "Robot",
    width: 340,
    inputs: [{ id: "program", label: "FANUC Program", pinType: "JSON", kind: "target" }],
    outputs: [
      { id: "action", label: "OMY3M Trajectory", pinType: "ACTION", kind: "source" },
      { id: "preview", label: "Retarget Preview", pinType: "JSON", kind: "source" },
      { id: "summary", label: "Retarget Summary", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "FANUC → OMY3M 리타게팅",
      positionScale: 0.25,
      targetOrigin: "0.35,0,0.25",
      targetAxisSigns: "1,-1,1",
      orientationOffsetDeg: "0,0,0",
      previewOnly: true,
      solveIk: true,
      requireAllIk: true,
      enableExecution: false,
      ikSeed: "0,-0.5,0.8,0,0.4,0",
      ikMaxIterations: 180,
      ikPositionToleranceM: 0.008,
      ikOrientationToleranceDeg: 8,
    },
  },

  fanuc_omy3m_isaaclab_export: {
    type: "fanuc_omy3m_isaaclab_export",
    label: "FANUC OMY3M IsaacLab 에피소드",
    category: "Dataset",
    width: 360,
    inputs: [
      { id: "prompt", label: "Instruction", pinType: "TEXT", kind: "target" },
      { id: "action", label: "OMY3M Trajectory", pinType: "ACTION", kind: "target" },
      { id: "result", label: "Execution Result", pinType: "JSON", kind: "target" },
    ],
    outputs: [
      { id: "dataset", label: "IsaacLab Dataset", pinType: "DATASET", kind: "source" },
      { id: "summary", label: "Episode Summary", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "FANUC OMY3M IsaacLab 에피소드",
      datasetPath: "isaaclab_datasets/omy3m_fanuc",
      episodeName: "fanuc_s05proc_001",
      instruction: "FANUC S05PROC trajectory retargeted to OMY3M",
      isaacLabBridgeRoot: "C:\\vlagraph_isaaclab_bridge",
      isaacLabTask: "Isaac-OMY3M-FANUC-Imitation-v0",
    },
  },

  print: {
    type: "print",
    label: "Print Node",
    category: "Output",
    width: 250,
    inputs: [{ id: "input", label: "Input", pinType: "ANY", kind: "target" }],
    outputs: [],
    defaults: {
      label: "Print Node",
    },
  },

  dataset_upload: {
    type: "dataset_upload",
    label: "Dataset Upload Node",
    category: "Dataset",
    width: 270,
    inputs: [],
    outputs: [
      { id: "dataset", label: "Dataset", pinType: "DATASET", kind: "source" },
      { id: "initialSettings", label: "Initial LS", pinType: "DATASET", kind: "source" },
    ],
    defaults: {
      label: "Dataset Upload Node",
      datasetPath: "",
      datasetFormat: "jsonl",
      initialSettingsPath: "",
      initialSettingsFormat: "ls",
    },
  },

  dataset_preview: {
    type: "dataset_preview",
    label: "Dataset Preview Node",
    category: "Preview",
    width: 260,
    inputs: [{ id: "dataset", label: "Dataset", pinType: "DATASET", kind: "target" }],
    outputs: [],
    defaults: {
      label: "Dataset Preview Node",
      maxRows: 3,
    },
  },

  model_sync: {
    type: "model_sync",
    label: "Model Sync Node",
    category: "Model",
    width: 320,
    inputs: [
      { id: "sourcePath", label: "Source Path", pinType: "TEXT", kind: "target" },
    ],
    outputs: [
      { id: "model", label: "Model", pinType: "MODEL", kind: "source" },
      { id: "status", label: "Status", pinType: "JSON", kind: "source" },
    ],
    defaults: {
      label: "Model Sync Node",
      sourcePath: "/host_downloads/Cosmos-Reason2-2B",
      targetName: "Cosmos-Reason2-2B",
      overwrite: false,
      requiredFiles: "config.json",
    },
  },

  openvla_finetune: {
    type: "openvla_finetune",
    label: "OpenVLA FineTune Node",
    category: "Train",
    width: 280,
    inputs: [{ id: "dataset", label: "Dataset", pinType: "DATASET", kind: "target" }],
    outputs: [{ id: "model", label: "Model", pinType: "MODEL", kind: "source" }],
    defaults: {
      label: "OpenVLA FineTune Node",
      baseModel: "openvla/openvla-7b",
      outputName: "openvla-exp",
      epochs: 1,
      batchSize: 2,
      learningRate: 0.0001,
      loraEnabled: true,
    },
  },
  
  speech_to_text: {
    type: "speech_to_text",
    label: "Speech To Text Node",
    category: "Audio",
    width: 280,

    inputs: [
      { id: "audio", label: "Audio", pinType: "FILE", kind: "target" },
    ],

    outputs: [
      { id: "text", label: "Text", pinType: "TEXT", kind: "source" },
      { id: "segments", label: "Segments", pinType: "JSON", kind: "source" },
      { id: "language", label: "Language", pinType: "TEXT", kind: "source" },
    ],

    defaults: {
      label: "Speech To Text Node",
      audioPath: "",
      provider: "whisper",
      language: "auto",
      task: "transcribe",
      temperature: 0,
      returnTimestamps: true,
    },
  },

  text_to_speech: {
    type: "text_to_speech",
    label: "Text To Speech Node",
    category: "Audio",
    width: 280,

    inputs: [
      { id: "text", label: "Text", pinType: "TEXT", kind: "target" },
    ],

    outputs: [
      { id: "audio", label: "Audio", pinType: "FILE", kind: "source" },
    ],

    defaults: {
      label: "Text To Speech Node",
      text: "",
      provider: "stub",
      language: "ko",
      voice: "default",
    },
  },
};

for (const type of ["fanuc_m710_prepare", "scene_scan", "fanuc_snapshot_read", "fanuc_usd_geometry_read",
  "fanuc_world_context", "execution_guard", "robot_execute", "fanuc_usd_proxy_build"]) {
  NODE_DEF_BY_TYPE[type].inputs.push({ id: "robot_config", label: "Robot Settings", pinType: "JSON", kind: "target" });
}

export function pinColor(pinType: PinType) {
  switch (pinType) {
    case "EXEC":
      return "#f59e0b";
    case "NUMBER":
      return "#22c55e";
    case "IMAGE":
      return "#3b82f6";
    case "JSON":
      return "#8b5cf6";
    case "DATASET":
      return "#10b981";
    case "MODEL":
      return "#ec4899";
    case "ACTION":
      return "#c084fc";
    case "TEXT":
      return "#60a5fa";
    case "FILE":
      return "#f97316";
    case "ANY":
    default:
      return "#94a3b8";
  }
}

export function nodeColor(type: string) {
  if (type === "print") return "#1a1333";
  if (type === "action_decode") return "#0f2a25";
  if (type === "g1_skill_decode") return "#10251f";
  if (type === "safety_clamp") return "#2a1a0f";
  if (type === "robot_execute") return "#2a1020";
  if (type.includes("spatial") || type.includes("scene_")) return "#10231f";
  if (type.includes("rgbd") || type.includes("realsense") || type.includes("mediapipe")) return "#0e2430";
  if (type.includes("motion")) return "#10251f";
  if (type.includes("sensor")) return "#0f1f2f";
  if (type.includes("speech") || type.includes("audio")) return "#24140a";
  if (type.includes("model")) return "#24122a";
  if (type.includes("dataset")) return "#0c1f1a";
  if (type.includes("finetune")) return "#131a2a";
  if (type.includes("unity")) return "#102438";
  if (type.includes("gr00t")) return "#09231d";
  if (type.includes("openvla")) return "#0b1220";
  return "#0b1220";
}

export function getPinDef(
  nodeType: string,
  handleId: string | null | undefined,
  kind: "target" | "source"
): PinDef | null {
  const def = NODE_DEF_BY_TYPE[nodeType];
  if (!def || !handleId) return null;

  const arr = kind === "target" ? def.inputs : def.outputs;
  return arr.find((p) => p.id === handleId) ?? null;
}

export function pinTypeOfNodeHandle(
  nodeType: string,
  handleId: string | null | undefined,
  kind: "target" | "source"
): PinType | null {
  return getPinDef(nodeType, handleId, kind)?.pinType ?? null;
}

export function isCompatiblePinType(sourceType: PinType | null, targetType: PinType | null) {
  if (!sourceType || !targetType) return false;
  if (sourceType === "ANY" || targetType === "ANY") return true;
  if (sourceType === targetType) return true;

  // 필요하면 여기에 느슨한 호환 규칙 추가
  // 예: ACTION -> JSON 직접 허용하고 싶으면 아래 활성화
  // if (sourceType === "ACTION" && targetType === "JSON") return true;

  return false;
}

export function edgeKindFromPins(sourceType: PinType | null, targetType: PinType | null) {
  if (sourceType === "EXEC" && targetType === "EXEC") return "exec";
  return "data";
}

export function canConnect(params: {
  sourceNodeType?: string | null;
  sourceHandle?: string | null;
  targetNodeType?: string | null;
  targetHandle?: string | null;
}) {
  const sourcePinType = pinTypeOfNodeHandle(
    params.sourceNodeType ?? "",
    params.sourceHandle,
    "source"
  );
  const targetPinType = pinTypeOfNodeHandle(
    params.targetNodeType ?? "",
    params.targetHandle,
    "target"
  );

  if (!sourcePinType || !targetPinType) return false;
  return isCompatiblePinType(sourcePinType, targetPinType);
}

export function createNodeData(type: string, extra?: Record<string, any>) {
  const def = NODE_DEF_BY_TYPE[type];
  return {
    ...(def?.defaults ?? {}),
    ...(extra ?? {}),
  };
}
