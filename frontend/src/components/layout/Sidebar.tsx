import React from "react";
import type { CanvasGraph as GraphListItem } from "../../graphWorkspace";

type PaletteItem = {
  label: string;
  description: string;
  type: string;
};

type PaletteGroup = {
  title: string;
  items: PaletteItem[];
};

const groups: PaletteGroup[] = [
  {
    title: "COMMON",
    items: [
      { label: "Prompt", description: "Text instruction input", type: "prompt" },
      { label: "로봇 설정", description: "로봇 모델·자동/수동 프로파일·연결 설정", type: "robot_setup" },
      { label: "Print", description: "Debug output node", type: "print" },
    ],
  },
  {
    title: "INFERENCE",
    items: [
      { label: "Image Input", description: "Single RGB image path or URL", type: "image_input" },
      { label: "Camera Sensor", description: "Browser camera capture with image output", type: "sensor_camera" },
      { label: "Remote Image", description: "Snapshot, MJPEG, or RTSP image source", type: "sensor_remote_image" },
      { label: "Remote LiDAR", description: "Unity or remote JSON LiDAR scan source", type: "sensor_remote_lidar" },
      { label: "Fused Sensor", description: "Combine camera or remote image with LiDAR", type: "sensor_fused" },
      { label: "RealSense D435i", description: "Intel RGB-D capture with aligned depth", type: "realsense_d435i_sensor" },
      { label: "MediaPipe Pose", description: "2D pose landmarks plus annotated image", type: "mediapipe_pose" },
      { label: "RGB-D Pose Fusion", description: "Fuse pose landmarks with depth into 3D motion", type: "rgbd_pose_fusion" },
      { label: "Video Motion", description: "Upload a human motion video", type: "video_motion_input" },
      { label: "Motion Extract", description: "Extract pose trace and skill label", type: "motion_extract" },
      { label: "Motion Retarget", description: "Convert motion trace to humanoid action", type: "motion_retarget" },
      { label: "Robot State", description: "EEF pose / gripper state", type: "robot_state" },
      { label: "Robot State (Isaac Sim)", description: "Latest Isaac Sim state plus manual GR00T scene init", type: "unity_state" },
      { label: "OpenVLA Infer", description: "Run action inference", type: "openvla_infer" },
      { label: "NVIDIA GR00T", description: "Run Isaac GR00T action inference", type: "gr00t_infer" },
      { label: "G1 Skill Decode", description: "Map prompt/action to Unitree G1 skill commands", type: "g1_skill_decode" },
      { label: "OMY3M 6DOF Action", description: "Create safe six-axis joint targets", type: "omy3m_joint_action" },
      { label: "FANUC LS Parser", description: "Parse FANUC LS motion programs", type: "fanuc_ls_parse" },
      { label: "FANUC M-710 Execute", description: "Prepare safe J1-J6 targets for the M-710 Isaac Sim bridge", type: "fanuc_m710_prepare" },
      { label: "FANUC → OMY3M", description: "Retarget FANUC poses for OMY3M preview", type: "fanuc_omy3m_retarget" },
    ],
  },
  {
    title: "DATASET",
    items: [
      { label: "Dataset Upload", description: "Training dataset file path", type: "dataset_upload" },
      { label: "Dataset Preview", description: "Preview first rows", type: "dataset_preview" },
      { label: "OMY3M IsaacLab Record", description: "Save synchronized state, action, and wrist image", type: "omy3m_isaaclab_record" },
      { label: "FANUC OMY3M Dataset", description: "Export retargeted trajectory as an IsaacLab episode", type: "fanuc_omy3m_isaaclab_export" },
    ],
  },
  {
    title: "SPATIAL",
    items: [
      { label: "Scene Scan", description: "Read Isaac Sim scene objects", type: "scene_scan" },
      { label: "World State Build", description: "Normalize observed geometry to meters and Z-up", type: "world_state_build" },
      { label: "FANUC State Read", description: "Read M-710 joints and external axis without moving", type: "fanuc_state_read" },
      { label: "FANUC Snapshot", description: "Capture joint velocities and simulator clock", type: "fanuc_snapshot_read" },
      { label: "FANUC USD Geometry", description: "Read authored collision shapes and joint coverage", type: "fanuc_usd_geometry_read" },
      { label: "FANUC USD Proxy", description: "Build robot collision proxies and report missing geometry", type: "fanuc_usd_proxy_build" },
      { label: "FANUC Collection Context", description: "Compare saved Collection observations with the current scene", type: "fanuc_world_context" },
      { label: "FANUC Prediction Handoff", description: "Bind geometry sweep to context; report unavailable physical prediction", type: "fanuc_world_prediction" },
      { label: "FANUC Preflight", description: "Check joint trajectory limits and source fidelity", type: "fanuc_trajectory_validate" },
      { label: "FANUC Collision", description: "Check configured link capsules and obstacle boxes", type: "fanuc_collision_check" },
      { label: "Execution Observe", description: "Read completion for a tracked FANUC command", type: "execution_observe" },
      { label: "Execution Guard", description: "Check report binding and execution scope", type: "execution_guard" },
      { label: "Scene Reset", description: "Set scene memory namespace", type: "scene_context_reset" },
      { label: "Memory Save", description: "Append spatial JSONL record", type: "spatial_memory_save" },
      { label: "RAG Retrieve", description: "Find relevant spatial memory", type: "spatial_rag_retrieve" },
      { label: "Context Builder", description: "Build prompt spatial context", type: "spatial_context_builder" },
    ],
  },
  {
    title: "MODEL",
    items: [
      { label: "Model Sync", description: "Copy a downloaded model into Docker volume", type: "model_sync" },
    ],
  },
  {
    title: "AUDIO",
    items: [
      { label: "Speech To Text", description: "음성을 원문 텍스트로 추출하거나 영어로 번역", type: "speech_to_text" },
      { label: "Text To Speech", description: "텍스트를 음성으로 변환", type: "text_to_speech" },
    ],
  },
  {
    title: "TRAINING",
    items: [
      { label: "OpenVLA FineTune", description: "Start LoRA/full tune job", type: "openvla_finetune" },
    ],
  },
];

export default function Sidebar({
  onAddNode,
  graphs,
  activeGraphId,
  onSelectGraph,
  disabled = false,
}: {
  onAddNode: (type: string) => void;
  disabled?: boolean;
  graphs: GraphListItem[];
  activeGraphId: string | null;
  onSelectGraph: (graphId: string) => void;
}) {
  const [activeTab, setActiveTab] = React.useState<"nodes" | "graphs">("nodes");

  return (
    <aside className="sidebar">
      <div className="brand">
        <h1>VLA Graph</h1>
        <p>Build OPENVLA tuning and inference pipelines</p>
      </div>

      <div className="sidebar-tabs">
        <button
          className={activeTab === "nodes" ? "active" : ""}
          onClick={() => setActiveTab("nodes")}
        >
          Nodes
        </button>
        <button
          className={activeTab === "graphs" ? "active" : ""}
          onClick={() => setActiveTab("graphs")}
        >
          Graphs
        </button>
      </div>

      {activeTab === "graphs" ? (
        <section className="palette-group">
          <div className="group-title">CANVAS GRAPHS</div>
          {graphs.length === 0 ? (
            <div className="graph-list-empty">No graphs on the canvas.</div>
          ) : (
            <div className="graph-list">
              {graphs.map((graph) => (
                <button
                  key={graph.id}
                  className={`graph-list-card${graph.id === activeGraphId ? " active" : ""}`}
                  aria-pressed={graph.id === activeGraphId}
                  onClick={() => onSelectGraph(graph.id)}
                >
                  <span className="graph-list-title">{graph.label}</span>
                  <span className="graph-list-description">{graph.description}</span>
                  <span className="graph-list-meta">
                    {graph.nodeCount} nodes / {graph.edgeCount} edges{graph.bundled ? " / Bundle" : ""}
                  </span>
                </button>
              ))}
            </div>
          )}
        </section>
      ) : (
        groups.map((group) => (
          <section key={group.title} className="palette-group">
            <div className="group-title">{group.title}</div>
            {group.items.map((item) => (
              <div key={item.type} className="palette-card">
                <div className="palette-label">{item.label}</div>
                <div className="palette-description">{item.description}</div>
                <button className="add-btn" disabled={disabled} onClick={() => onAddNode(item.type)}>
                  Add Node
                </button>
              </div>
            ))}
          </section>
        ))
      )}
    </aside>
  );
}
