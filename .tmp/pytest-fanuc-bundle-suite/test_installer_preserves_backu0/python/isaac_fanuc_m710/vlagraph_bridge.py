class Bridge:
    def __init__(self):
        self._last_state_ok_at = 0.0

        # VLAGRAPH_WORLD_MODEL_ADAPTER_V1
        from .vlagraph_world_model import attach_world_model
        attach_world_model(self)
