import React from "react";
import type { Node } from "reactflow";
import { isSensorNodeType } from "../../nodeDefs";

function prettyValue(value: unknown) {
  if (value == null) return "";
  if (typeof value === "string") return value;
  return JSON.stringify(value, null, 2);
}

function isBooleanKey(key: string) {
  return [
    "enabled",
    "requireInitialSnapshot",
    "requireWorldContext",
    "dryRun",
    "loraEnabled",
    "returnTimestamps",
    "requireImage",
    "overwrite",
    "includeFrames",
    "fullBodyRetarget",
    "mockWhenUnavailable",
    "omniverseWaitForResponse",
    "rootTranslationEnabled",
    "rootPitchEnabled",
    "terminated",
    "success",
    "requireImage",
    "saveFailed",
    "previewOnly",
    "solveIk",
    "requireAllIk",
    "enableExecution",
  ].includes(key);
}

function isNumberKey(key: string) {
  return [
    "maxStepSec",
    "marginM",
    "maxSamples",
    "maxSnapshotAgeSec",
    "maxColliders",
    "paddingM",
    "maxDelta",
    "epochs",
    "batchSize",
    "learningRate",
    "simulateSpeed",
    "maxRows",
    "temperature",
    "captureWidth",
    "captureHeight",
    "fps",
    "frameTimeoutMs",
    "warmupFrames",
    "depthMinMeters",
    "depthMaxMeters",
    "modelComplexity",
    "minDetectionConfidence",
    "minTrackingConfidence",
    "minPoseVisibility",
    "depthSampleRadius",
    "maxMotionFrames",
    "requestTimeoutSec",
    "lidarObstacleDistance",
    "waypointSpeed",
    "actionHorizon",
    "unityTimeoutSec",
    "omniverseTimeoutSec",
    "initTimeoutSec",
    "timeoutSec",
    "everyNFrames",
    "maxFrames",
    "minVisibility",
    "stepScale",
    "turnScale",
    "durationSec",
    "secondsPerFrame",
    "maxPoseFrames",
    "poseScale",
    "rootLateralScale",
    "rootVerticalScale",
    "rootDepthScale",
    "rootImageXDepthScale",
    "rootMaxLateral",
    "rootMaxVertical",
    "rootMaxDepth",
    "rootMaxPitchDeg",
    "gripperPosition",
    "jointLimitRad",
    "reward",
    "positionScale",
    "ikMaxIterations",
    "ikPositionToleranceM",
    "ikOrientationToleranceDeg",
  ].includes(key);
}

function isJsonKey(key: string) {
  return ["robotState", "remoteHeaders", "remoteBody", "lidarSample"].includes(key);
}

function isRootMotionModeKey(nodeType: string | undefined, key: string) {
  return nodeType === "motion_retarget" && key === "rootMotionMode";
}
function isModeKey(nodeType: string | undefined, key: string) {
  return nodeType === "robot_execute" && key === "mode";
}

function isUnityCommandKey(nodeType: string | undefined, key: string) {
  return nodeType === "robot_execute" && key === "unityCommand";
}

function isUnityRobotTypeKey(nodeType: string | undefined, key: string) {
  return nodeType === "robot_execute" && key === "unityRobotType";
}

function isUnityControllerModeKey(nodeType: string | undefined, key: string) {
  return nodeType === "robot_execute" && key === "unityControllerMode";
}

function isSensorModeKey(nodeType: string | undefined, key: string) {
  return nodeType === "sensor_input" && key === "sourceMode";
}

function isSensorFacingModeKey(nodeType: string | undefined, key: string) {
  return isSensorNodeType(nodeType) && key === "cameraFacingMode";
}

function isSensorRemoteMethodKey(nodeType: string | undefined, key: string) {
  return isSensorNodeType(nodeType) && key === "remoteMethod";
}

function isSensorRemoteTransportKey(nodeType: string | undefined, key: string) {
  return isSensorNodeType(nodeType) && key === "remoteImageTransport";
}

function isSensorLidarFormatKey(nodeType: string | undefined, key: string) {
  return isSensorNodeType(nodeType) && key === "lidarFormat";
}

function isSensorRtspTransportKey(nodeType: string | undefined, key: string) {
  return isSensorNodeType(nodeType) && key === "rtspTransport";
}

function isModelNameKey(nodeType: string | undefined, key: string) {
  return nodeType === "openvla_infer" && key === "modelName";
}

function isOpenVlaActionFormatKey(nodeType: string | undefined, key: string) {
  return nodeType === "openvla_infer" && key === "actionFormat";
}

function isOpenVlaUnityCommandKey(nodeType: string | undefined, key: string) {
  return nodeType === "openvla_infer" && key === "unityCommand";
}

function isGr00tInferenceModeKey(nodeType: string | undefined, key: string) {
  return nodeType === "gr00t_infer" && key === "inferenceMode";
}

function isGr00tActionFormatKey(nodeType: string | undefined, key: string) {
  return nodeType === "gr00t_infer" && key === "actionFormat";
}

function isGr00tUnityCommandKey(nodeType: string | undefined, key: string) {
  return nodeType === "gr00t_infer" && key === "unityCommand";
}

function isSpeechProviderKey(nodeType: string | undefined, key: string) {
  return nodeType === "speech_to_text" && key === "provider";
}

function isSpeechLanguageKey(nodeType: string | undefined, key: string) {
  return nodeType === "speech_to_text" && key === "language";
}

function isSpeechTaskKey(nodeType: string | undefined, key: string) {
  return nodeType === "speech_to_text" && key === "task";
}

function isTTSProviderKey(nodeType: string | undefined, key: string) {
  return nodeType === "text_to_speech" && key === "provider";
}

function isTTSLanguageKey(nodeType: string | undefined, key: string) {
  return nodeType === "text_to_speech" && key === "language";
}

function isTTSVoiceKey(nodeType: string | undefined, key: string) {
  return nodeType === "text_to_speech" && key === "voice";
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

function speechProviderLabel(provider: unknown) {
  const normalized = String(provider ?? "whisper");
  if (normalized === "stub") return "Stub Demo";
  if (normalized === "faster-whisper") return "Faster Whisper";
  return "Whisper";
}

function ttsProviderLabel(provider: unknown) {
  const normalized = String(provider ?? "stub");
  if (normalized === "stub") return "Stub Demo";
  if (normalized === "real") return "Real TTS";
  return "Stub Demo";
}

function ttsLanguageLabel(language: unknown) {
  const normalized = String(language ?? "ko");
  if (normalized === "ko") return "한국어";
  if (normalized === "en") return "영어";
  if (normalized === "ja") return "일본어";
  return "한국어";
}

function ttsVoiceLabel(voice: unknown) {
  const normalized = String(voice ?? "default");
  if (normalized === "default") return "기본 목소리";
  return normalized;
}

const UNITY_ROBOT_TYPE_OPTIONS = [
  "generic",
  "articulated_arm",
  "mobile_base",
  "humanoid",
];

const UNITY_CONTROLLER_MODE_OPTIONS: Record<string, string[]> = {
  generic: ["task", "cartesian", "joint", "base", "locomotion"],
  articulated_arm: ["joint", "cartesian", "task"],
  mobile_base: ["base", "task"],
  humanoid: ["locomotion", "task", "cartesian"],
};

const UNITY_COMMAND_OPTIONS: Record<string, string[]> = {
  generic: ["apply_action", "stop"],
  articulated_arm: [
    "apply_joint_targets",
    "apply_joint_deltas",
    "apply_joint_velocities",
    "stop",
  ],
  mobile_base: ["apply_base_velocity", "apply_base_waypoint", "stop"],
  humanoid: [
    "apply_humanoid_step",
    "apply_base_waypoint",
    "apply_patrol_control",
    "apply_action",
    "stop",
  ],
};

function unityRobotTypeLabel(robotType: unknown) {
  const normalized = String(robotType ?? "generic");
  if (normalized === "articulated_arm") return "articulated_arm";
  if (normalized === "mobile_base") return "mobile_base";
  if (normalized === "humanoid") return "humanoid";
  return "generic";
}

function unityControllerModeLabel(mode: unknown) {
  const normalized = String(mode ?? "task");
  if (normalized === "cartesian") return "cartesian";
  if (normalized === "joint") return "joint";
  if (normalized === "base") return "base";
  if (normalized === "locomotion") return "locomotion";
  return "task";
}

function unityCommandLabel(command: unknown) {
  const normalized = String(command ?? "apply_action");
  if (normalized === "apply_joint_targets") return "apply_joint_targets";
  if (normalized === "apply_joint_deltas") return "apply_joint_deltas";
  if (normalized === "apply_joint_velocities") return "apply_joint_velocities";
  if (normalized === "apply_base_velocity") return "apply_base_velocity";
  if (normalized === "apply_base_waypoint") return "apply_base_waypoint";
  if (normalized === "apply_humanoid_step") return "apply_humanoid_step";
  if (normalized === "apply_patrol_control") return "apply_patrol_control";
  if (normalized === "stop") return "stop";
  return "apply_action";
}

function unityControllerModesForRobotType(robotType: unknown) {
  const normalized = String(robotType ?? "generic");
  return UNITY_CONTROLLER_MODE_OPTIONS[normalized] ?? UNITY_CONTROLLER_MODE_OPTIONS.generic;
}

function unityCommandsForRobotType(robotType: unknown) {
  const normalized = String(robotType ?? "generic");
  return UNITY_COMMAND_OPTIONS[normalized] ?? UNITY_COMMAND_OPTIONS.generic;
}

function fieldLabel(nodeType: string | undefined, key: string) {
  if (nodeType === "robot_execute") {
    const robotLabels: Record<string, string> = {
      mode: "실행 모드",
      executor: "실행기 이름",
      robotType: "Robot Type",
      controllerMode: "Controller Mode",
      clientId: "State Client",
      unityEndpoint: "Unity WS Endpoint",
      unityScene: "Unity Scene",
      unityObject: "Unity Object",
      unityCommand: "Unity Command",
      unityRobotType: "Unity Robot Type",
      unityControllerMode: "Unity Controller Mode",
      unityTimeoutSec: "Unity Timeout (sec)",
      unityStateClientId: "Unity State Client",
      omniverseEndpoint: "Omniverse WS Endpoint",
      omniverseUrl: "Omniverse URL",
      omniverseStage: "Omniverse Stage",
      omniversePrimPath: "Omniverse Prim Path",
      omniverseCommand: "Omniverse Command",
      omniverseTimeoutSec: "Omniverse Timeout (sec)",
      omniverseWaitForResponse: "Omniverse Wait For Response",
      rosBridgeUrl: "ROS Bridge URL",
      rosTopic: "ROS Topic",
      rosRobotName: "ROS Robot Name",
      rosFrameId: "ROS Frame ID",
      rosMsgType: "ROS Msg Type",
    };
    if (robotLabels[key]) {
      return robotLabels[key];
    }
  }

  if (nodeType === "unity_state") {
    const unityStateLabels: Record<string, string> = {
      clientId: "Client ID",
      omniverseEndpoint: "Omniverse WS Endpoint",
      initTimeoutSec: "Init Timeout (sec)",
    };
    if (unityStateLabels[key]) {
      return unityStateLabels[key];
    }
  }

  if (nodeType === "openvla_infer") {
    const inferLabels: Record<string, string> = {
      modelName: "모델",
      actionFormat: "액션 포맷",
      unityCommand: "Unity 명령 힌트",
      waypointSpeed: "Waypoint 속도",
    };
    if (inferLabels[key]) {
      return inferLabels[key];
    }
  }

  if (nodeType === "gr00t_infer") {
    const gr00tLabels: Record<string, string> = {
      modelName: "GR00T 모델",
      endpoint: "GR00T Endpoint",
      embodimentTag: "Embodiment Tag",
      inferenceMode: "추론 모드",
      actionHorizon: "Action Horizon",
      actionFormat: "액션 포맷",
      unityCommand: "Unity 명령 힌트",
      waypointSpeed: "Waypoint 속도",
      requireImage: "이미지 필수",
    };
    if (gr00tLabels[key]) {
      return gr00tLabels[key];
    }
  }

  if (nodeType === "model_sync") {
    const modelLabels: Record<string, string> = {
      sourcePath: "소스 경로",
      targetName: "대상 모델 이름",
      overwrite: "덮어쓰기",
      requiredFiles: "필수 파일",
    };
    if (modelLabels[key]) {
      return modelLabels[key];
    }
  }

  if (nodeType === "speech_to_text") {
    const speechLabels: Record<string, string> = {
      audioPath: "오디오 파일",
      provider: "STT 엔진",
      language: "입력 언어",
      task: "출력 모드",
      temperature: "샘플링 온도",
      returnTimestamps: "구간 타임스탬프",
    };
    if (speechLabels[key]) {
      return speechLabels[key];
    }
  }

  if (nodeType === "text_to_speech") {
    const ttsLabels: Record<string, string> = {
      text: "텍스트 입력",
      provider: "TTS 엔진",
      language: "출력 언어",
      voice: "목소리",
    };
    if (ttsLabels[key]) {
      return ttsLabels[key];
    }
  }

  if (
    nodeType === "sensor_remote_image" ||
    nodeType === "sensor_remote_lidar" ||
    nodeType === "sensor_fused" ||
    nodeType === "sensor_input"
  ) {
    const remoteImageLabels: Record<string, string> = {
      remoteImageUrl: "원격 이미지 URL",
      remoteLidarUrl: "원격 LiDAR URL",
      remoteImageTransport: "트랜스포트 타입",
      remoteMethod: "HTTP 메서드",
      remoteHeaders: "요청 헤더",
      remoteBody: "요청 바디",
      requestTimeoutSec: "요청 타임아웃 (초)",
      lidarFormat: "LiDAR 포맷",
      lidarObstacleDistance: "장애물 거리 기준 (m)",
      rtspTransport: "RTSP transport",
      ffmpegPath: "FFmpeg 경로",
    };
    if (remoteImageLabels[key]) {
      return remoteImageLabels[key];
    }
  }

  if (nodeType === "video_motion_input" || nodeType === "motion_extract" || nodeType === "motion_retarget") {
    const motionLabels: Record<string, string> = {
      videoPath: "영상 파일",
      motionLabel: "동작 라벨",
      prompt: "프롬프트",
      everyNFrames: "샘플 간격",
      maxFrames: "최대 샘플 프레임",
      minVisibility: "최소 landmark visibility",
      includeFrames: "전체 프레임 포함",
      outputName: "출력 JSON 이름",
      speed: "속도",
      stepScale: "이동 크기",
      turnScale: "회전 크기",
      durationSec: "동작 시간 (초)",
      fullBodyRetarget: "전신 포즈 시퀀스 생성",
      jointSmoothing: "G1 관절 smoothing",
      jointMaxDeltaRad: "프레임당 최대 관절 변화",
      jointAverageWindow: "관절 평균 프레임",
      maxPoseFrames: "포즈 샘플 프레임",
      poseScale: "포즈 회전 배율",
    };
    if (motionLabels[key]) {
      return motionLabels[key];
    }
  }

  if (nodeType === "rgbd_pose_fusion") {
    const rgbdLabels: Record<string, string> = {
      depthSampleRadius: "Depth 샘플 반경",
      minPoseVisibility: "최소 landmark visibility",
      landmarkSmoothing: "Landmark smoothing",
      landmarkMaxDelta: "프레임당 최대 landmark 변화",
      maxMotionFrames: "모션 히스토리 프레임",
      motionLabel: "동작 라벨",
      prompt: "프롬프트",
    };
    if (rgbdLabels[key]) {
      return rgbdLabels[key];
    }
  }

  return key;
}

function fieldHelp(nodeType: string | undefined, key: string, value: unknown) {
  if (nodeType === "robot_execute") {
    if (key === "mode") {
      const mode = String(value ?? "simulate").toLowerCase();
      if (mode === "unity") {
        return "Unity WebSocket 서버로 액션 명령을 보내고 ACK 응답을 기다립니다.";
      }
      if (mode === "omniverse") {
        return "Omniverse WebSocket 서버로 액션 명령을 보내고 ACK 응답을 기다립니다.";
      }
      if (mode === "ros") {
        return "ROS Bridge 쪽 publish payload를 구성해 전송하는 모드입니다.";
      }
      return "내장 시뮬레이터 기준으로 액션을 검증하거나 데모 실행하는 모드입니다.";
    }

    if (key === "unityEndpoint") {
      return "Unity 제어 서버가 열어둔 WebSocket 주소입니다. 예: ws://host.docker.internal:9081/robot";
    }

    if (key === "unityCommand") {
      return "기본 명령 이름입니다. 아래 추천 프리셋에서 고르거나 custom 문자열로 직접 입력할 수 있습니다. 액션 payload 안의 command/unityCommand 값이 있으면 런타임에서 그 값으로 덮어쓸 수 있습니다.";
    }

    if (key === "unityRobotType") {
      return "로봇 프로필에 맞는 추천 명령/제어 모드를 자동으로 좁혀주기 때문에 먼저 이 값을 맞추는 편이 좋습니다.";
    }

    if (key === "unityControllerMode") {
      return "선택한 robot type에 맞는 추천 모드만 먼저 보여줍니다. 필요하면 액션 payload에서 controller_mode로 런타임 덮어쓰기도 가능합니다.";
    }

    if (key === "omniverseEndpoint") {
      return "Omniverse 쪽 제어 서버가 열어둔 WebSocket 주소입니다. 예: ws://host.docker.internal:9900/omniverse";
    }

    if (key === "omniverseUrl") {
      return "Omniverse/Nucleus 리소스 URL이나 stage 식별자를 기록하는 용도입니다.";
    }

    if (key === "omniversePrimPath") {
      return "명령을 적용할 Prim 경로입니다. 예: /World/GR00T_Humanoid";
    }

    if (key === "omniverseCommand") {
      return "Omniverse 서버 쪽에서 해석할 명령 이름입니다. 예: execute_policy";
    }

    if (key === "omniverseWaitForResponse") {
      return "끄면 명령만 전송하고 응답을 기다리지 않아 RGB-D 추종 루프가 Isaac 처리 시간에 덜 묶입니다.";
    }
  }

  if (nodeType === "openvla_infer") {
    if (key === "actionFormat") {
      return "비워두면 백엔드가 downstream Unity 명령을 보고 자동 보정합니다. apply_base_waypoint를 쓸 때는 unity_waypoint가 권장됩니다.";
    }

    if (key === "unityCommand") {
      return "비워두면 백엔드가 연결된 robot_execute(Unity) 노드의 unityCommand를 자동으로 상속합니다.";
    }

    if (key === "waypointSpeed") {
      return "waypoint 액션으로 변환될 때 Unity 쪽 이동 속도로 함께 전달됩니다.";
    }
  }

  if (nodeType === "unity_state") {
    if (key === "omniverseEndpoint") {
      return "Init GR00T Scene 버튼이 command=init을 보낼 Isaac Sim Omniverse WebSocket 주소입니다.";
    }

    if (key === "initTimeoutSec") {
      return "Init 버튼 클릭 후 Isaac Sim extension 응답을 기다릴 시간입니다.";
    }
  }

  if (nodeType === "gr00t_infer") {
    if (key === "endpoint") {
      return "비워두면 backend 환경변수 ROUTE_GR00T_REAL을 사용합니다. Docker의 HTTP 어댑터 서비스 gr00t_policy_adapter:9072 로 연결합니다.";
    }

    if (key === "modelName") {
      return "예: nvidia/GR00T-N1.7-3B. gr00t-stub 또는 gr00t-real을 넣으면 라우트 선택 힌트로도 사용합니다.";
    }

    if (key === "embodimentTag") {
      return "GR00T 체크포인트/데이터셋의 embodiment tag입니다. G1은 REAL_G1을 사용합니다.";
    }

    if (key === "inferenceMode") {
      return "정책 서버가 지원하는 실행 모드입니다. PyTorch, TensorRT, trt_full_pipeline 같은 값을 전달합니다.";
    }

    if (key === "actionHorizon") {
      return "GR00T가 한 번에 예측할 action chunk 길이입니다. 서버가 지원하지 않으면 무시될 수 있습니다.";
    }

    if (key === "actionFormat") {
      return "Unity waypoint/patrol/humanoid step으로 후처리할 힌트입니다. 비워두면 action payload를 그대로 전달합니다.";
    }

    if (key === "unityCommand") {
      return "Robot Execute(Unity) 쪽 명령과 맞춰두면 GR00T 출력이 해당 명령에 맞는 payload로 보정됩니다.";
    }

    if (key === "requireImage") {
      return "켜면 이미지 입력 없이 실행할 때 백엔드가 먼저 오류를 반환합니다.";
    }
  }

  if (nodeType === "model_sync") {
    if (key === "sourcePath") {
      return "backend 컨테이너 안에서 보이는 모델 폴더 경로입니다. Windows 다운로드 폴더는 /host_downloads로 mount됩니다.";
    }
    if (key === "targetName") {
      return "Docker volume /models 아래에 만들 폴더 이름입니다. 예: Cosmos-Reason2-2B 또는 GR00T-N1.7-3B";
    }
    if (key === "overwrite") {
      return "false면 대상 폴더가 이미 있을 때 복사하지 않고 검증만 합니다.";
    }
    if (key === "requiredFiles") {
      return "복사 후 확인할 파일 목록입니다. 쉼표로 구분합니다. 예: config.json,processor_config.json,statistics.json";
    }
  }

  if (
    nodeType === "sensor_remote_image" ||
    nodeType === "sensor_remote_lidar" ||
    nodeType === "sensor_fused" ||
    nodeType === "sensor_input"
  ) {
    if (key === "remoteImageUrl") {
      return "Unity/Omniverse, MJPEG, RTSP, 또는 JSON 이미지 제공 엔드포인트 URL을 입력합니다.";
    }
    if (key === "remoteImageTransport") {
      return "snapshot: 단일 이미지/JSON 응답, mjpeg: MJPEG 스트림, rtsp: RTSP 단일 프레임";
    }
    if (key === "remoteMethod") {
      return "GET 또는 POST로 이미지/JSON 요청을 전송합니다. POST를 사용할 때 remoteBody를 함께 설정하세요.";
    }
    if (key === "remoteHeaders") {
      return "Unity/Omniverse 서버에 전달할 추가 HTTP 헤더(JSON 객체).";
    }
    if (key === "remoteBody") {
      return "POST 요청 시 전송할 JSON 바디입니다. Unity/Omniverse가 JSON으로 이미지 payload를 반환할 수 있습니다.";
    }
    if (key === "requestTimeoutSec") {
      return "원격 이미지 요청의 타임아웃 시간(초)입니다.";
    }
    if (key === "remoteLidarUrl") {
      return "Unity LiDAR WebSocket이 서버에 저장한 최신 스캔은 기본적으로 /api/unity/lidar에서 읽습니다.";
    }
    if (key === "lidarFormat") {
      return "auto는 ranges/points/points_flat 구조를 감지해 laser_scan 또는 pointcloud로 정규화합니다.";
    }
    if (key === "lidarObstacleDistance") {
      return "이 거리 이내의 LiDAR 포인트를 가까운 장애물로 집계합니다.";
    }
    if (key === "rtspTransport") {
      return "RTSP에서 사용할 transport 방식입니다(tcp 또는 udp).";
    }
    if (key === "ffmpegPath") {
      return "ffmpeg 실행 파일 경로입니다. RTSP 캡처에 사용됩니다.";
    }
  }

  if (nodeType !== "speech_to_text") {
    return "";
  }

  if (key === "task") {
    return String(value ?? "transcribe") === "translate"
      ? "입력 음성을 영어 문장으로 번역해 다음 노드로 넘깁니다."
      : "입력 음성을 원문 언어 그대로 텍스트로 옮깁니다.";
  }

  if (key === "language") {
    return String(value ?? "auto") === "auto"
      ? "Whisper가 입력 언어를 자동으로 감지합니다."
      : `입력 음성을 ${speechLanguageLabel(value)}로 가정합니다.`;
  }

  if (key === "provider") {
    if (String(value ?? "whisper") === "stub") {
      return "Stub는 데모용 고정 응답입니다. 실제 인식과 번역은 Whisper를 사용하세요.";
    }
    if (String(value ?? "whisper") === "faster-whisper") {
      return "현재 서버에서는 Whisper와 같은 경로로 처리됩니다.";
    }
    return "실제 음성 인식과 영문 번역은 Whisper 엔진으로 처리합니다.";
  }

  if (key === "audioPath") {
    return "Upload Audio 또는 마이크 녹음으로 자동 연결할 수 있습니다.";
  }

  return "";
}

function shouldShowField(
  nodeType: string | undefined,
  key: string,
  data: Record<string, unknown>
) {
  if (nodeType !== "robot_execute") {
    return true;
  }

  const mode = String(data.mode ?? "simulate").toLowerCase();
  const commonKeys = new Set(["executor", "mode", "robotType", "controllerMode", "clientId"]);
  const simulateKeys = new Set(["simulateName", "simulateSpeed", "dryRun"]);
  const unityKeys = new Set([
    "unityEndpoint",
    "unityScene",
    "unityObject",
    "unityCommand",
    "unityRobotType",
    "unityControllerMode",
    "unityTimeoutSec",
    "unityStateClientId",
  ]);
  const omniverseKeys = new Set([
    "omniverseEndpoint",
    "omniverseUrl",
    "omniverseStage",
    "omniversePrimPath",
    "omniverseCommand",
    "omniverseTimeoutSec",
    "omniverseWaitForResponse",
    "rootTranslationEnabled",
    "rootPitchEnabled",
  ]);
  const rosKeys = new Set([
    "rosBridgeUrl",
    "rosTopic",
    "rosRobotName",
    "rosFrameId",
    "rosMsgType",
  ]);

  if (commonKeys.has(key)) {
    return true;
  }

  if (mode === "unity") {
    return unityKeys.has(key);
  }

  if (mode === "omniverse") {
    return omniverseKeys.has(key);
  }

  if (mode === "ros") {
    return rosKeys.has(key);
  }

  return simulateKeys.has(key);
}

function parseValueByKey(key: string, raw: string) {
  if (isBooleanKey(key)) {
    return raw === "true";
  }

  if (isNumberKey(key)) {
    const n = Number(raw);
    return Number.isNaN(n) ? 0 : n;
  }

  if (isJsonKey(key)) {
    try {
      return JSON.parse(raw);
    } catch {
      return raw;
    }
  }

  return raw;
}

function renderFieldInput(
  nodeType: string | undefined,
  key: string,
  value: unknown,
  data: Record<string, unknown>,
  nodeId: string,
  onNodeDataChange: (nodeId: string, key: string, value: unknown) => void
) {
  if (nodeType === "fanuc_usd_geometry_read" && key === "source") {
    return (
      <select value={String(value ?? "bridge")} onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}>
        <option value="bridge">Isaac Sim 현재 USD 장면</option>
        <option value="file">백엔드에서 접근 가능한 USD 파일</option>
      </select>
    );
  }
  if (nodeType === "execution_guard" && key === "validationPolicy") {
    return (
      <select value={String(value ?? "full")} onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}>
        <option value="full">전체 검증 — 미검증 항목이 있으면 차단</option>
        <option value="static_simulation">정적 검사 후 Isaac Sim 실행 — 충돌·물리 예측 미포함</option>
      </select>
    );
  }
  if (isRootMotionModeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "in_place")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="in_place">in_place - feet locked / pelvis vertical</option>
        <option value="locomotion">locomotion - root translation</option>
        <option value="auto">auto</option>
      </select>
    );
  }
  if (isSensorModeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "camera")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="camera">camera</option>
        <option value="remote_image">remote_image</option>
        <option value="remote_lidar">remote_lidar</option>
        <option value="fused">fused</option>
      </select>
    );
  }

  if (isSensorFacingModeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "environment")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="environment">environment</option>
        <option value="user">user</option>
      </select>
    );
  }

  if (isSensorRemoteMethodKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "GET")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="GET">GET</option>
        <option value="POST">POST</option>
      </select>
    );
  }

  if (isSensorRemoteTransportKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "snapshot")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="snapshot">snapshot</option>
        <option value="mjpeg">mjpeg</option>
        <option value="rtsp">rtsp</option>
      </select>
    );
  }

  if (isSensorLidarFormatKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "auto")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="auto">auto</option>
        <option value="pointcloud">pointcloud</option>
        <option value="laser_scan">laser_scan</option>
      </select>
    );
  }

  if (isSensorRtspTransportKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "tcp")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="tcp">tcp</option>
        <option value="udp">udp</option>
      </select>
    );
  }

  if (isModeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "simulate")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="simulate">simulate</option>
        <option value="unity">unity</option>
        <option value="omniverse">omniverse</option>
        <option value="ros">ROS(ROBOT)</option>
      </select>
    );
  }

  if (isUnityRobotTypeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "generic")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        {UNITY_ROBOT_TYPE_OPTIONS.map((option) => (
          <option key={option} value={option}>
            {unityRobotTypeLabel(option)}
          </option>
        ))}
      </select>
    );
  }

  if (isUnityControllerModeKey(nodeType, key)) {
    const robotType = String(data.unityRobotType ?? "generic");
    const options = unityControllerModesForRobotType(robotType);
    const currentValue = String(value ?? options[0] ?? "task");
    const selectValue = options.includes(currentValue) ? currentValue : "__custom__";

    return (
      <div style={{ display: "grid", gap: 6 }}>
        <select
          value={selectValue}
          onChange={(e) => {
            const nextValue = e.target.value;
            if (nextValue === "__custom__") {
              onNodeDataChange(nodeId, key, options.includes(currentValue) ? "" : currentValue);
              return;
            }
            onNodeDataChange(nodeId, key, nextValue);
          }}
        >
          {options.map((option) => (
            <option key={option} value={option}>
              {unityControllerModeLabel(option)}
            </option>
          ))}
          <option value="__custom__">custom</option>
        </select>
        {selectValue === "__custom__" && (
          <input
            value={currentValue}
            placeholder="custom controller mode"
            onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
          />
        )}
      </div>
    );
  }

  if (isUnityCommandKey(nodeType, key)) {
    const robotType = String(data.unityRobotType ?? "generic");
    const options = unityCommandsForRobotType(robotType);
    const currentValue = String(value ?? options[0] ?? "apply_action");
    const selectValue = options.includes(currentValue) ? currentValue : "__custom__";

    return (
      <div style={{ display: "grid", gap: 6 }}>
        <select
          value={selectValue}
          onChange={(e) => {
            const nextValue = e.target.value;
            if (nextValue === "__custom__") {
              onNodeDataChange(nodeId, key, options.includes(currentValue) ? "" : currentValue);
              return;
            }
            onNodeDataChange(nodeId, key, nextValue);
          }}
        >
          {options.map((option) => (
            <option key={option} value={option}>
              {unityCommandLabel(option)}
            </option>
          ))}
          <option value="__custom__">custom</option>
        </select>
        {selectValue === "__custom__" && (
          <input
            value={currentValue}
            placeholder="custom unity command"
            onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
          />
        )}
      </div>
    );
  }

  if (isModelNameKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "openvla-stub")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="openvla-stub">openvla-stub</option>
        <option value="openvla-real">openvla-real</option>
      </select>
    );
  }

  if (isOpenVlaActionFormatKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="">auto</option>
        <option value="delta_action">delta_action</option>
        <option value="unity_waypoint">unity_waypoint</option>
      </select>
    );
  }

  if (isOpenVlaUnityCommandKey(nodeType, key)) {
    return (
      <input
        value={String(value ?? "")}
        placeholder="auto from robot_execute"
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      />
    );
  }

  if (isGr00tInferenceModeKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "pytorch")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="pytorch">pytorch</option>
        <option value="torch_compile">torch_compile</option>
        <option value="tensorrt">tensorrt</option>
        <option value="trt_full_pipeline">trt_full_pipeline</option>
      </select>
    );
  }

  if (isGr00tActionFormatKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="">auto</option>
        <option value="delta_action">delta_action</option>
        <option value="unity_waypoint">unity_waypoint</option>
        <option value="patrol_control">patrol_control</option>
        <option value="humanoid_step">humanoid_step</option>
      </select>
    );
  }

  if (isGr00tUnityCommandKey(nodeType, key)) {
    return (
      <input
        value={String(value ?? "")}
        placeholder="apply_humanoid_step / apply_base_waypoint"
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      />
    );
  }

  if (isSpeechProviderKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "whisper")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="whisper">{speechProviderLabel("whisper")}</option>
        <option value="faster-whisper">{speechProviderLabel("faster-whisper")}</option>
        <option value="stub">{speechProviderLabel("stub")}</option>
      </select>
    );
  }

  if (isSpeechLanguageKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "ko")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="auto">{speechLanguageLabel("auto")}</option>
        <option value="ko">{speechLanguageLabel("ko")}</option>
        <option value="en">{speechLanguageLabel("en")}</option>
        <option value="ja">{speechLanguageLabel("ja")}</option>
      </select>
    );
  }

  if (isSpeechTaskKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "transcribe")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="transcribe">{speechTaskLabel("transcribe")}</option>
        <option value="translate">{speechTaskLabel("translate")}</option>
      </select>
    );
  }

  if (isTTSProviderKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "stub")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="stub">{ttsProviderLabel("stub")}</option>
        <option value="real">{ttsProviderLabel("real")}</option>
      </select>
    );
  }

  if (isTTSLanguageKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "ko")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="ko">{ttsLanguageLabel("ko")}</option>
        <option value="en">{ttsLanguageLabel("en")}</option>
        <option value="ja">{ttsLanguageLabel("ja")}</option>
      </select>
    );
  }

  if (isTTSVoiceKey(nodeType, key)) {
    return (
      <select
        value={String(value ?? "default")}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
      >
        <option value="default">{ttsVoiceLabel("default")}</option>
      </select>
    );
  }

  if (isBooleanKey(key)) {
    return (
      <select
        value={String(Boolean(value))}
        onChange={(e) => onNodeDataChange(nodeId, key, e.target.value === "true")}
      >
        <option value="true">true</option>
        <option value="false">false</option>
      </select>
    );
  }

  if (isNumberKey(key)) {
    return (
      <input
        type="number"
        step={
          key === "learningRate" ||
          key === "maxDelta" ||
          key === "simulateSpeed" ||
          key === "lidarObstacleDistance"
            ? "0.01"
            : "1"
        }
        value={String(value ?? "")}
        onChange={(e) => onNodeDataChange(nodeId, key, parseValueByKey(key, e.target.value))}
      />
    );
  }

  const isTextArea =
    isJsonKey(key) ||
    typeof value === "object" ||
    String(value ?? "").length > 40;

  if (isTextArea) {
    return (
      <textarea
        value={prettyValue(value)}
        onChange={(e) => onNodeDataChange(nodeId, key, parseValueByKey(key, e.target.value))}
        rows={4}
      />
    );
  }

  return (
    <input
      value={String(value ?? "")}
      onChange={(e) => onNodeDataChange(nodeId, key, e.target.value)}
    />
  );
}

export default function Inspector({
  selectedNode,
  nodeCount,
  edgeCount,
  logs,
  onNodeDataChange,
}: {
  selectedNode: Node | null;
  nodeCount: number;
  edgeCount: number;
  logs: string[];
  onNodeDataChange: (nodeId: string, key: string, value: unknown) => void;
}) {
  const data = (selectedNode?.data ?? {}) as Record<string, unknown>;
  const nodeType = selectedNode?.type;

  const filteredEntries = Object.entries(data).filter(
    ([key]) =>
      key !== "label" &&
      key !== "status" &&
      key !== "lastSummary" &&
      key !== "lastDetail" &&
      key !== "lastRunAt" &&
      key !== "lastRawLidarJson" &&
      key !== "lastLidarOutputJson" &&
      key !== "lastLidarPointPreview" &&
      key !== "onRemove" &&
      key !== "onNodeDataChange" &&
      key !== "onIsaacGr00tInit" &&
      key !== "initStatus" &&
      key !== "initMessage" &&
      key !== "initEndpoint" &&
      key !== "initPrimPath" &&
      key !== "__ui" &&
      key !== "capturedFrame" &&
      key !== "lastImageFrame" &&
      key !== "lastDepthImageFrame" &&
      key !== "lastVisionJson" &&
      key !== "lastTracePath" &&
      key !== "lastIsaacLabTrainCommand" &&
      key !== "lastMotionFrameCount" &&
      shouldShowField(nodeType, key, data)
  );

  return (
    <aside className="inspector">
      <div className="inspector-heading">
        <div>
          <span className="workspace-eyebrow">PROPERTIES</span>
          <h2>노드 속성</h2>
        </div>
        <span className="inspector-heading-icon">×</span>
      </div>

      <div className="panel-card">
        <div className="panel-title">선택한 노드</div>
        <div className="panel-body strong">
          {selectedNode ? (selectedNode.data?.label as string) ?? selectedNode.id : "노드를 선택하세요"}
        </div>
      </div>

      {selectedNode && (
        <div className="panel-card">
          <div className="panel-title">노드 설정</div>
              {(data.lastSummary || data.lastDetail) && (
            <div className={`run-summary-box tone-${String(data.status ?? "idle")}`}>
              <div className="run-summary-title">{String(data.lastSummary ?? "최근 실행 결과")}</div>
              <div className="run-summary-detail">{String(data.lastDetail ?? "")}</div>
            </div>
          )}
          {nodeType === "speech_to_text" && (
            <div className="field-hint" style={{ marginTop: 10 }}>
              현재 STT 노드는 원문 추출과 영문 번역 중 하나를 선택해 사용할 수 있습니다.
            </div>
          )}
          {nodeType === "robot_execute" && (
            <div className="field-hint" style={{ marginTop: 10 }}>
              현재 실행 모드: {String(data.mode ?? "simulate")}
            </div>
          )}
          {(nodeType === "motion_extract" ||
            nodeType === "fanuc_omy3m_isaaclab_export") &&
            data.lastTracePath && (
            <div className="run-summary-box tone-success" style={{ marginTop: 10 }}>
              <div className="run-summary-title">IsaacLab 학습 준비 완료</div>
              <div className="run-summary-detail">
                trace: {String(data.lastTracePath)}
                {data.lastMotionFrameCount ? ` / frames: ${String(data.lastMotionFrameCount)}` : ""}
              </div>
              {data.lastIsaacLabTrainCommand && (
                <>
                  <textarea
                    readOnly
                    value={String(data.lastIsaacLabTrainCommand)}
                    style={{
                      width: "100%",
                      minHeight: 86,
                      marginTop: 8,
                      resize: "vertical",
                      fontFamily: "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace",
                      fontSize: 11,
                    }}
                  />
                  <button
                    type="button"
                    className="secondary-btn"
                    style={{ marginTop: 8 }}
                    onClick={() => navigator.clipboard?.writeText(String(data.lastIsaacLabTrainCommand))}
                  >
                    Copy IsaacLab Command
                  </button>
                </>
              )}
            </div>
          )}
          <div className="form-grid">
            <label className="field-row">
              <span>label</span>
              <input
                value={String(data.label ?? "")}
                onChange={(e) => onNodeDataChange(selectedNode.id, "label", e.target.value)}
              />
            </label>

            {filteredEntries.map(([key, value]) => (
              <label key={key} className="field-row">
                <span>{fieldLabel(nodeType, key)}</span>
                {renderFieldInput(nodeType, key, value, data, selectedNode.id, onNodeDataChange)}
                {fieldHelp(nodeType, key, value) && (
                  <div className="field-hint">{fieldHelp(nodeType, key, value)}</div>
                )}
              </label>
            ))}
          </div>
        </div>
      )}

      <div className="panel-card">
        <div className="panel-title">그래프 요약</div>
        <div className="panel-body">
          <div>노드 {nodeCount}개</div>
          <div>연결 {edgeCount}개</div>
        </div>
      </div>

      <div className="panel-card">
        <div className="panel-title">실행 로그</div>
        <div className="panel-body log-box">
          {logs.length === 0
            ? <div>그래프가 준비되었습니다. 실행을 기다리는 중입니다.</div>
            : logs.map((log, i) => <div key={i}>{log}</div>)}
        </div>
      </div>
    </aside>
  );
}


