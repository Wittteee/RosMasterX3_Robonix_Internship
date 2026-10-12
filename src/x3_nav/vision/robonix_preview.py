from __future__ import annotations

import importlib
import json
import sys
from pathlib import Path
from typing import Any


class RobonixPreview:
    """Use the reference package's generated request and in-process Provider API.

    This matches its move_base_cli.ProviderApi.preview integration. The package
    and generated bindings must already be installed in the running environment.
    """

    def __init__(self, provider_dir: str | None = None) -> None:
        if provider_dir:
            sys.path.insert(0, str(Path(provider_dir).resolve()))
        self.provider = importlib.import_module("rosmaster_x3_bridge.main")
        self.bindings = importlib.import_module("rosmaster_x3_bridge_mcp")

    def preview(self) -> dict[str, Any]:
        response = self.provider.preview_move_options(
            self.bindings.PreviewMoveOptions_Request(max_options=5)
        )
        return json.loads(response.result_json.data)

    def close(self) -> None:
        # Disconnect without calling adapter.close(), which may have stop side effects.
        bridge = getattr(self.provider, "bridge", None)
        if bridge is not None:
            bridge.ros.terminate()
            self.provider.bridge = None
