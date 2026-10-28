import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HOST = os.environ.get("OMNIVERSE_LAUNCHER_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNIVERSE_LAUNCHER_PORT", "9188"))
TOKEN = os.environ.get("OMNIVERSE_LAUNCHER_TOKEN", "")
LAUNCH_SCRIPT = Path(os.environ.get("OMNIVERSE_LAUNCH_SCRIPT", ""))


class LauncherHandler(BaseHTTPRequestHandler):
    def _write_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/health":
            self._write_json(200, {"ok": True, "service": "omniverse-launcher"})
            return
        self._write_json(404, {"ok": False, "message": "Not found"})

    def do_POST(self) -> None:
        if self.path != "/launch":
            self._write_json(404, {"ok": False, "message": "Not found"})
            return

        if not TOKEN:
            self._write_json(503, {"ok": False, "message": "Launcher token is not configured"})
            return

        authorization = self.headers.get("Authorization", "")
        if authorization != f"Bearer {TOKEN}":
            self._write_json(401, {"ok": False, "message": "Unauthorized launcher request"})
            return

        if not LAUNCH_SCRIPT.exists():
            self._write_json(503, {"ok": False, "message": "Launch script does not exist", "path": str(LAUNCH_SCRIPT)})
            return

        powershell = (
            str(Path(os.environ["SYSTEMROOT"]) / "System32" / "WindowsPowerShell" / "v1.0" / "powershell.exe")
            if os.environ.get("SYSTEMROOT")
            else "powershell.exe"
        )
        if not Path(powershell).exists():
            powershell = "powershell.exe"

        try:
            subprocess.Popen(
                [powershell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(LAUNCH_SCRIPT)],
                cwd=str(LAUNCH_SCRIPT.parent),
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0,
            )
        except Exception as error:
            self._write_json(500, {"ok": False, "message": f"Failed to start Isaac Sim: {error}"})
            return

        self._write_json(200, {"ok": True, "message": "시뮬레이션 폴더를 열었습니다."})

    def log_message(self, format: str, *args) -> None:
        print(f"[omniverse-launcher] {format % args}", flush=True)


if __name__ == "__main__":
    server = ThreadingHTTPServer((HOST, PORT), LauncherHandler)
    print(f"[omniverse-launcher] listening on http://{HOST}:{PORT}", flush=True)
    server.serve_forever()
