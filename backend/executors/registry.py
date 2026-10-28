# backend/executors/registry.py

from backend.executors.common_nodes import (
    exec_prompt,
    exec_image_input,
    exec_robot_state,
    exec_unity_state,
    exec_print,
)

from backend.executors.dataset_nodes import (
    exec_dataset_upload,
    exec_dataset_preview,
)
from backend.executors.sensor_nodes import exec_sensor_input
from backend.executors.rgbd_pose_nodes import (
    exec_mediapipe_pose,
    exec_realsense_d435i_sensor,
    exec_rgbd_pose_fusion,
)
from backend.executors.motion_nodes import (
    exec_motion_extract,
    exec_motion_retarget,
    exec_video_motion_input,
)
from backend.executors.g1_skill_nodes import exec_g1_skill_decode
from backend.executors.robot_setup_nodes import exec_robot_setup
from backend.executors.amr_nodes import (
    exec_robot_observations_read, exec_amr_path_plan, exec_amr_navigate, exec_amr_status, exec_amr_stop,
)
from backend.executors.multi_robot_nodes import (
    exec_multi_robot_context, exec_multi_robot_infer, exec_robot_candidate_select, exec_fanuc_policy_prepare,
)
from backend.executors.omy3m_nodes import (
    exec_omy3m_isaaclab_record,
    exec_omy3m_joint_action,
)
from backend.executors.fanuc_nodes import (
    exec_fanuc_ls_parse,
    exec_fanuc_m710_prepare,
    exec_fanuc_omy3m_retarget,
    exec_fanuc_omy3m_isaaclab_export,
)

from backend.executors.vla_nodes import (
    exec_openvla_infer,
    exec_gr00t_infer,
    exec_action_decode,
    exec_safety_clamp,
    exec_robot_execute,   # ← 핵심
    exec_openvla_finetune,
)
from backend.executors.audio_nodes import exec_speech_to_text, exec_text_to_speech
from backend.executors.model_nodes import exec_model_sync
from backend.executors.spatial_nodes import (
    exec_scene_context_reset,
    exec_scene_scan,
    exec_spatial_context_builder,
    exec_spatial_memory_save,
    exec_spatial_rag_retrieve,
)
from backend.executors.world_model_nodes import (
    exec_fanuc_world_context, exec_fanuc_world_prediction,
    exec_execution_guard, exec_execution_observe, exec_fanuc_collision_check, exec_fanuc_snapshot_read, exec_fanuc_state_read, exec_fanuc_trajectory_validate, exec_fanuc_usd_geometry_read, exec_fanuc_usd_proxy_build, exec_world_state_build,
)


NODE_EXECUTORS = {
    # -------------------------
    # Basic Nodes
    # -------------------------
    "prompt": exec_prompt,
    "image_input": exec_image_input,
    "robot_state": exec_robot_state,
    "unity_state": exec_unity_state,
    "print": exec_print,

    # -------------------------
    # Dataset
    # -------------------------
    "dataset_upload": exec_dataset_upload,
    "dataset_preview": exec_dataset_preview,
    "model_sync": exec_model_sync,
    "sensor_input": exec_sensor_input,
    "sensor_camera": exec_sensor_input,
    "sensor_remote_image": exec_sensor_input,
    "sensor_remote_lidar": exec_sensor_input,
    "sensor_fused": exec_sensor_input,
    "realsense_d435i_sensor": exec_realsense_d435i_sensor,
    "mediapipe_pose": exec_mediapipe_pose,
    "rgbd_pose_fusion": exec_rgbd_pose_fusion,
    "video_motion_input": exec_video_motion_input,
    "motion_extract": exec_motion_extract,
    "motion_retarget": exec_motion_retarget,
    "g1_skill_decode": exec_g1_skill_decode,
    "robot_setup": exec_robot_setup,
    "omy3m_joint_action": exec_omy3m_joint_action,
    "omy3m_isaaclab_record": exec_omy3m_isaaclab_record,
    "fanuc_ls_parse": exec_fanuc_ls_parse,
    "fanuc_m710_prepare": exec_fanuc_m710_prepare,
    "fanuc_omy3m_retarget": exec_fanuc_omy3m_retarget,
    "fanuc_omy3m_isaaclab_export": exec_fanuc_omy3m_isaaclab_export,
    "scene_scan": exec_scene_scan,
    "world_state_build": exec_world_state_build,
    "robot_observations_read": exec_robot_observations_read,
    "amr_path_plan": exec_amr_path_plan,
    "amr_navigate": exec_amr_navigate,
    "amr_status": exec_amr_status,
    "amr_stop": exec_amr_stop,
    "multi_robot_context": exec_multi_robot_context,
    "multi_robot_infer": exec_multi_robot_infer,
    "robot_candidate_select": exec_robot_candidate_select,
    "fanuc_policy_prepare": exec_fanuc_policy_prepare,
    "fanuc_state_read": exec_fanuc_state_read,
    "fanuc_snapshot_read": exec_fanuc_snapshot_read,
    "execution_observe": exec_execution_observe,
    "execution_guard": exec_execution_guard,
    "fanuc_trajectory_validate": exec_fanuc_trajectory_validate,
    "fanuc_collision_check": exec_fanuc_collision_check,
    "fanuc_usd_geometry_read": exec_fanuc_usd_geometry_read,
    "fanuc_usd_proxy_build": exec_fanuc_usd_proxy_build,
    "fanuc_world_context": exec_fanuc_world_context,
    "fanuc_world_prediction": exec_fanuc_world_prediction,
    "scene_context_reset": exec_scene_context_reset,
    "spatial_memory_save": exec_spatial_memory_save,
    "spatial_rag_retrieve": exec_spatial_rag_retrieve,
    "spatial_context_builder": exec_spatial_context_builder,

    # -------------------------
    # VLA Pipeline
    # -------------------------
    "openvla_infer": exec_openvla_infer,
    "gr00t_infer": exec_gr00t_infer,
    "action_decode": exec_action_decode,
    "safety_clamp": exec_safety_clamp,

    # 🔥 핵심 노드
    "robot_execute": exec_robot_execute,

    # -------------------------
    # Training
    # -------------------------
    "openvla_finetune": exec_openvla_finetune,

    # -------------------------
    # STT
    # -------------------------
    "speech_to_text": exec_speech_to_text,
    "text_to_speech": exec_text_to_speech,
}
