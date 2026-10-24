#!/usr/bin/env python3
"""Live lidar viewer: /scan as a top-down point cloud in a web page, plus a map of everything seen.

    python3 rl/scan_viewer.py            then open http://localhost:8765
    scripts/start_sim.sh ... --scan-view (runs it in the sim container)

The latest scan is drawn round the robot, coloured by distance. When the robot's pose is known
(in Webots: /sim/position and /imu), every return is also dropped into a 5 cm grid in the world
frame, so walking around builds up a map of the room. Without a pose (the real robot has no
odometry yet) the view stays in the robot's frame and only the live scan is shown.

No dependencies beyond rclpy and numpy; the page is served by the standard library.
"""

import argparse
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64MultiArray

CELL = 0.05          # m, map grid
MAX_CELLS = 60000


class ScanState(Node):
    def __init__(self):
        super().__init__("bob_scan_viewer")
        self.lock = threading.Lock()
        self.scan = np.zeros((0, 2))
        self.ranges = np.zeros(0)
        self.pose = None              # (x, y, yaw) in the world, when known
        self.position, self.yaw = None, None
        self.cells = {}               # (i, j) -> hits
        self.scans, self.last_stamp, self.rate = 0, None, 0.0
        self.create_subscription(LaserScan, "/scan", self.on_scan, 5)
        self.create_subscription(PointStamped, "/sim/position", self.on_position, 5)
        self.create_subscription(Imu, "/imu", self.on_imu, 5)
        self.floor = np.zeros((0, 3))     # floor sensor hits: x, y (world or body frame), height
        self.create_subscription(Float64MultiArray, "/tof/points", self.on_floor, 5)

    def on_position(self, msg):
        self.position = (msg.point.x, msg.point.y)

    def on_floor(self, msg):
        pts = np.asarray(msg.data, dtype=float).reshape(-1, 3)
        with self.lock:
            if self.position is not None and self.yaw is not None and len(pts):
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                xy = pts[:, :2] @ np.array([[c, s], [-s, c]]) + self.position
                pts = np.column_stack([xy, pts[:, 2]])
            self.floor = pts

    def on_imu(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        local = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.last_stamp is not None and stamp > self.last_stamp:
                self.rate = 0.8 * self.rate + 0.2 / (stamp - self.last_stamp)
            self.last_stamp = stamp
            self.scans += 1
            self.ranges = r[ok]
            if self.position is not None and self.yaw is not None:
                x, y, yaw = *self.position, self.yaw
                self.pose = (x, y, yaw)
                c, s = math.cos(yaw), math.sin(yaw)
                world = local @ np.array([[c, s], [-s, c]]) + [x, y]
                self.scan = world
                if len(self.cells) < MAX_CELLS:
                    for i, j in np.floor(world / CELL).astype(int).tolist():
                        self.cells[(i, j)] = self.cells.get((i, j), 0) + 1
            else:
                self.pose = None
                self.scan = local

    def snapshot(self):
        with self.lock:
            cells = [[i, j, n] for (i, j), n in self.cells.items()]
            return {
                "pose": self.pose,
                "scan": np.round(self.scan, 3).tolist(),
                "ranges": np.round(self.ranges, 3).tolist(),
                "map": cells, "cell": CELL,
                "scans": self.scans, "rate": round(self.rate, 1),
                "floor": np.round(self.floor, 3).tolist(),
            }

    def clear(self):
        with self.lock:
            self.cells = {}


def handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, body, kind):
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/scan.json"):
                self.send(json.dumps(state.snapshot()), "application/json")
            else:
                self.send(PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            if self.path == "/clear":
                state.clear()
            self.send("{}", "application/json")
    return Handler


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Bob's Lidar</title>
<style>
:root { --bg:#f6f4f0; --panel:#ffffff; --ink:#22211f; --muted:#6f6b64; --grid:#e3ded5; --map:#4a5a70;
        --robot:#d9622b; --line:#d8d2c7; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#151618; --panel:#1f2124; --ink:#ecebe8; --muted:#9a978f; --grid:#2a2d31; --map:#8aa2c4;
  --robot:#ff8a4c; --line:#34373c; } }
:root[data-theme="dark"] { --bg:#151618; --panel:#1f2124; --ink:#ecebe8; --muted:#9a978f; --grid:#2a2d31;
  --map:#8aa2c4; --robot:#ff8a4c; --line:#34373c; }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--ink); font:14px/1.4 system-ui, -apple-system, sans-serif;
       height:100vh; display:flex; flex-direction:column; }
header { display:flex; flex-wrap:wrap; gap:8px 18px; align-items:center; padding:10px 16px;
         background:var(--panel); border-bottom:1px solid var(--line); }
h1 { font-size:15px; margin:0 8px 0 0; }
.stat { color:var(--muted); font-variant-numeric:tabular-nums; }
.stat b { color:var(--ink); font-weight:600; }
.spacer { flex:1; }
button, label { font:inherit; color:var(--ink); }
button { background:transparent; border:1px solid var(--line); border-radius:6px; padding:4px 10px; cursor:pointer; }
button:hover { border-color:var(--muted); }
#wrap { flex:1; position:relative; min-height:0; }
canvas { position:absolute; inset:0; width:100%; height:100%; touch-action:none; }
#legend { position:absolute; left:16px; bottom:14px; color:var(--muted); font-size:12px; display:flex; gap:10px;
          align-items:center; background:var(--panel); padding:6px 10px; border-radius:6px; border:1px solid var(--line); }
#ramp { width:110px; height:8px; border-radius:4px; background:linear-gradient(90deg,#e8423f,#f2a93b,#7cc65a,#3aa6c9,#6a5acd); }
.dot { width:10px; height:10px; border-radius:2px; background:var(--map); display:inline-block; }
</style></head>
<body>
<header>
  <h1>Bob's lidar</h1>
  <span class="stat">points <b id="n">-</b></span>
  <span class="stat">nearest <b id="near">-</b></span>
  <span class="stat">scan rate <b id="rate">-</b></span>
  <span class="stat">map <b id="cells">-</b></span>
  <span class="stat" id="frame"></span>
  <span class="spacer"></span>
  <label><input type="checkbox" id="follow" checked> follow</label>
  <label><input type="checkbox" id="showmap" checked> map</label>
  <button id="clear">clear map</button>
</header>
<div id="wrap"><canvas id="c"></canvas>
  <div id="legend"><span>lidar: 0 m</span><span id="ramp"></span><span>6 m+</span><span class="dot"></span><span>seen before</span>
    <span>| floor sensor: <b style="color:#3c78c8">drop</b> <b style="color:#50c83c">level</b> <b style="color:#ff5032">step up</b></span></div>
</div>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
let data = null, scale = 80, center = [0, 0], drag = null;
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function rampColor(d) {
  const stops = [[0,[232,66,63]],[1.5,[242,169,59]],[3,[124,198,90]],[4.5,[58,166,201]],[6,[106,90,205]]];
  d = Math.min(Math.max(d, 0), 6);
  for (let i = 1; i < stops.length; i++) if (d <= stops[i][0]) {
    const [a, ca] = stops[i-1], [b, cb] = stops[i], t = (d - a) / (b - a);
    return `rgb(${ca.map((v, k) => Math.round(v + t * (cb[k] - v))).join(',')})`;
  }
  return 'rgb(106,90,205)';
}
function resize() { const r = cv.getBoundingClientRect(), dpr = devicePixelRatio || 1;
  cv.width = r.width * dpr; cv.height = r.height * dpr; ctx.setTransform(dpr, 0, 0, dpr, 0, 0); draw(); }
function toScreen(x, y) { const r = cv.getBoundingClientRect();
  return [r.width / 2 + (x - center[0]) * scale, r.height / 2 - (y - center[1]) * scale]; }
function draw() {
  const r = cv.getBoundingClientRect();
  ctx.fillStyle = css('--bg'); ctx.fillRect(0, 0, r.width, r.height);
  // 1 m grid
  ctx.strokeStyle = css('--grid'); ctx.lineWidth = 1; ctx.beginPath();
  const x0 = Math.floor(center[0] - r.width / 2 / scale), x1 = Math.ceil(center[0] + r.width / 2 / scale);
  const y0 = Math.floor(center[1] - r.height / 2 / scale), y1 = Math.ceil(center[1] + r.height / 2 / scale);
  for (let x = x0; x <= x1; x++) { const [sx] = toScreen(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, r.height); }
  for (let y = y0; y <= y1; y++) { const [, sy] = toScreen(0, y); ctx.moveTo(0, sy); ctx.lineTo(r.width, sy); }
  ctx.stroke();
  if (!data) return;
  const pose = data.pose || [0, 0, 0];
  if (document.getElementById('showmap').checked && data.map.length) {
    ctx.fillStyle = css('--map'); const s = Math.max(1.5, data.cell * scale);
    for (const [i, j, n] of data.map) { if (n < 2) continue;
      const [sx, sy] = toScreen((i + 0.5) * data.cell, (j + 0.5) * data.cell);
      ctx.globalAlpha = Math.min(0.25 + n / 20, 0.85); ctx.fillRect(sx - s / 2, sy - s / 2, s, s); }
    ctx.globalAlpha = 1;
  }
  // range rings round the robot
  const [rx, ry] = toScreen(pose[0], pose[1]);
  ctx.strokeStyle = css('--line'); ctx.setLineDash([3, 4]);
  for (const m of [1, 2, 4]) { ctx.beginPath(); ctx.arc(rx, ry, m * scale, 0, 2 * Math.PI); ctx.stroke(); }
  ctx.setLineDash([]);
  // live scan
  data.scan.forEach(([x, y], k) => { const [sx, sy] = toScreen(x, y);
    ctx.fillStyle = rampColor(data.ranges[k]); ctx.fillRect(sx - 2, sy - 2, 4, 4); });
  // floor sensor: where its 64 zones land, coloured by the floor height there
  for (const [x, y, h] of (data.floor || [])) { const [sx, sy] = toScreen(x, y);
    const t = Math.max(-1, Math.min(1, h / 0.08));
    ctx.fillStyle = t >= 0 ? `rgba(${Math.round(80 + 175 * t)}, ${Math.round(200 - 120 * t)}, 60, 0.9)` : `rgba(60, 120, ${Math.round(200 - 100 * t)}, 0.9)`;
    ctx.beginPath(); ctx.arc(sx, sy, 3.5, 0, 2 * Math.PI); ctx.fill(); }
  // robot
  ctx.save(); ctx.translate(rx, ry); ctx.rotate(-pose[2]); ctx.fillStyle = css('--robot');
  const b = Math.max(6, 0.12 * scale);
  ctx.beginPath(); ctx.moveTo(b * 1.3, 0); ctx.lineTo(-b, b * 0.8); ctx.lineTo(-b * 0.5, 0); ctx.lineTo(-b, -b * 0.8);
  ctx.closePath(); ctx.fill(); ctx.restore();
}
async function poll() {
  try {
    data = await (await fetch('scan.json', {cache: 'no-store'})).json();
    if (document.getElementById('follow').checked && data.pose) center = [data.pose[0], data.pose[1]];
    if (!data.pose) center = [0, 0];
    document.getElementById('n').textContent = data.scan.length;
    document.getElementById('near').textContent = data.ranges.length ? Math.min(...data.ranges).toFixed(2) + ' m' : '-';
    document.getElementById('rate').textContent = data.rate + ' Hz';
    document.getElementById('cells').textContent = data.pose ? data.map.length + ' cells' : 'needs a pose';
    document.getElementById('frame').textContent = data.pose ? 'world frame' : 'robot frame (no pose)';
    draw();
  } catch (e) { document.getElementById('frame').textContent = 'waiting for the viewer...'; }
  setTimeout(poll, 150);
}
cv.addEventListener('wheel', e => { e.preventDefault(); scale = Math.min(400, Math.max(15, scale * Math.exp(-e.deltaY / 400))); draw(); }, {passive: false});
cv.addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY, ...center]; cv.setPointerCapture(e.pointerId); });
cv.addEventListener('pointermove', e => { if (!drag) return; document.getElementById('follow').checked = false;
  center = [drag[2] - (e.clientX - drag[0]) / scale, drag[3] + (e.clientY - drag[1]) / scale]; draw(); });
cv.addEventListener('pointerup', () => drag = null);
document.getElementById('clear').onclick = () => fetch('clear', {method: 'POST'});
document.getElementById('showmap').onchange = draw;
addEventListener('resize', resize); resize(); poll();
</script></body></html>
"""


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8765)
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    state = ScanState()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    state.get_logger().info(f"lidar viewer on http://localhost:{args.port}")
    try:
        rclpy.spin(state)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        state.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
