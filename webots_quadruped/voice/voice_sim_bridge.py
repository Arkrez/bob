#!/usr/bin/env python3
"""Local browser voice/text -> ROS 2 -> Webots simulation bridge.

Runs INSIDE the ROS 2 environment, not on the real robot. Never starts the
physical Servo2040 bridge. Bind to loopback by default; do not expose on LAN.
"""
import argparse
import json
import math
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINTS = {"1": 0, "3": 1, "4": 2}
MAX_DEGREES = 45.0
HTML = Path(__file__).with_name("voice_control.html")


class SimBridge(Node):
    def __init__(self):
        super().__init__("bob_voice_sim_bridge")
        self.publisher = self.create_publisher(
            Float64MultiArray, "/leg_position_controller/commands", 10
        )
        self.pose = [0.0] * 12
        self.lock = threading.Lock()

    def execute(self, phrase):
        cmd = phrase.lower().strip().replace("degrees", "degree")
        with self.lock:
            listeners = self.publisher.get_subscription_count()
            if not listeners:
                return {"ok": False, "message": "Webots bridge is not subscribed; no command sent."}
            if re.fullmatch(r"(reset|neutral|home|reset robot|reset joints)", cmd):
                updated = [0.0] * 12
            elif re.fullmatch(r"(stop|halt|freeze)", cmd):
                # Position-controlled motors hold the most recent target.
                return {"ok": True, "message": "Holding last commanded positions.", "pose_degrees": self._degrees()}
            else:
                # Intentionally precise grammar: no ambiguous natural language
                # joint mapping or unverified 'walking' gait is executed.
                match = re.fullmatch(
                    r"(?:move |set )?leg\s*([0-3])\s*(?:joint\s*)?(1|3|4)\s*"
                    r"(?:to|at)\s*(-?\d+(?:\.\d+)?)\s*(?:degree)?",
                    cmd,
                )
                if not match:
                    return {"ok": False, "message": (
                        "Try: 'set leg 0 joint 3 to 15 degrees', 'reset', or 'stop'. "
                        "Legs are numbered 0–3 until physical leg ordering is calibrated."
                    )}
                leg, joint, value = match.groups()
                value = float(value)
                if not math.isfinite(value) or abs(value) > MAX_DEGREES:
                    return {"ok": False, "message": "Target must be between -45 and +45 degrees."}
                updated = self.pose.copy()
                updated[int(leg) * 3 + JOINTS[joint]] = math.radians(value)
            msg = Float64MultiArray()
            msg.data = updated
            self.publisher.publish(msg)
            self.pose = updated
            return {"ok": True, "message": "Command published to Webots simulation topic.", "pose_degrees": self._degrees()}

    def _degrees(self):
        return [round(math.degrees(x), 2) for x in self.pose]


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/":
            self.send_error(404)
            return
        self._send(200, HTML.read_bytes(), "text/html; charset=utf-8")

    def do_POST(self):
        if self.path != "/command":
            self.send_error(404)
            return
        # Browser localhost Origin guard. No remote access.
        origin = self.headers.get("Origin")
        if origin not in (None, f"http://localhost:{self.server.server_port}",
                          f"http://127.0.0.1:{self.server.server_port}"):
            self._json(403, {"ok": False, "message": "Invalid origin"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 1 or size > 1024:
                raise ValueError("Request must be 1–1024 bytes")
            request = json.loads(self.rfile.read(size))
            if not isinstance(request, dict) or not isinstance(request.get("text"), str):
                raise ValueError("Expected JSON object with a text string")
            if len(request["text"]) > 160:
                raise ValueError("Command too long")
            result = self.server.bridge.execute(request["text"])
            self._json(200 if result["ok"] else 422, result)
        except (ValueError, json.JSONDecodeError) as exc:
            self._json(400, {"ok": False, "message": str(exc)})

    def _json(self, status, data):
        self._send(status, json.dumps(data).encode(), "application/json; charset=utf-8")

    def _send(self, status, payload, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt, *args):
        print("voice-ui:", fmt % args)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind interface (default localhost; do not expose publicly)")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    rclpy.init()
    node = SimBridge()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.bridge = node
    try:
        print(f"Voice control: http://localhost:{args.port}")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
