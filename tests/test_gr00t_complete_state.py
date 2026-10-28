import pytest

from services.gr00t_policy_adapter import gr00t_http_adapter as adapter


@pytest.fixture
def modalities(monkeypatch):
    monkeypatch.delenv("GR00T_EMBODIMENT_KEY", raising=False)
    monkeypatch.setattr(adapter, "_processor_modality_configs", lambda: {"fanuc_joint_space": {}, "another_robot": {}})
    monkeypatch.setattr(adapter, "_state_dim", lambda *args: 6)
    monkeypatch.setattr(adapter, "_stat_mean", lambda *args: [0.]*6)
    monkeypatch.setattr(adapter, "_horizon", lambda *args: 1)
    monkeypatch.setattr(adapter, "_modality_keys", lambda embodiment, modality, fallback:
                        {"video": ["ego_view"], "state": ["joint_positions"], "language": ["task"]}[modality])


@pytest.mark.parametrize("values", [None, [0.]*5, [0.]*7, [True]+[0.]*5, [float("nan")]+[0.]*5])
def test_strict_multi_robot_request_never_pads_or_truncates_missing_state(modalities, values):
    request = adapter.InferRequest(embodiment_tag="fanuc_joint_space", require_complete_state=True,
                                   state={"joint_positions": values})
    with pytest.raises(ValueError, match="Complete measured state"):
        adapter._build_observation(request)


def test_strict_request_uses_the_requested_embodiment_and_exact_measured_values(modalities):
    values = [.1, .2, .3, .4, .5, .6]
    request = adapter.InferRequest(embodiment_tag="fanuc_joint_space", require_complete_state=True,
                                   state={"joint_positions": values})
    observation, debug = adapter._build_observation(request)
    assert debug["embodiment_key"] == "fanuc_joint_space"
    assert observation["state"]["joint_positions"][0, 0].tolist() == pytest.approx(values)


def test_unknown_or_server_overridden_embodiment_is_rejected(modalities, monkeypatch):
    for tag in ("unknown_robot", "fanuc_joint_space"):
        monkeypatch.setenv("GR00T_EMBODIMENT_KEY", "another_robot")
        request = adapter.InferRequest(embodiment_tag=tag, require_complete_state=True,
                                       state={"joint_positions": [0.]*6})
        with pytest.raises(ValueError, match="embodiment"):
            adapter._build_observation(request)


def test_strict_camera_capture_cannot_substitute_black_image(monkeypatch):
    monkeypatch.setattr(adapter, "_load_image_bytes", lambda _: None)
    with pytest.raises(ValueError, match="blank image"):
        adapter._image_to_observation_array("missing", "", 1, strict=True)


def test_legacy_request_keeps_its_existing_padding_behavior(modalities):
    request = adapter.InferRequest(embodiment_tag="fanuc_joint_space", state={"joint_positions": [.5]})
    observation, _ = adapter._build_observation(request)
    assert observation["state"]["joint_positions"][0, 0].tolist() == [.5, 0., 0., 0., 0., 0.]
