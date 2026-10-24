"""Record what Bob does in Webots, for plot_watch.py to draw: path, heading, navigator status, the
target and a frame from his camera.

    scripts/sim_ros.sh python3 /workspace/rl/sim_watch.py 60          record 60 s
    rl/.venv/bin/python rl/plot_watch.py                               -> rl/runs/sim_watch.png
"""

import json
import math
import pathlib
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

OUT = pathlib.Path(__file__).resolve().parent / "runs"


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    n = rclpy.create_node("sim_watch")
    st = {"pos": None, "yaw": 0.0, "status": "", "target": None, "image": None}
    n.create_subscription(PointStamped, "/sim/position", lambda m: st.__setitem__("pos", (m.point.x, m.point.y, m.point.z)), 5)
    n.create_subscription(PointStamped, "/sim/target", lambda m: st.__setitem__("target", (m.point.x, m.point.y)), 5)
    n.create_subscription(String, "/nav/status", lambda m: st.__setitem__("status", m.data), 5)

    def imu(m):
        q = m.orientation
        st["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    n.create_subscription(Imu, "/imu", imu, 5)
    n.create_subscription(Image, "/camera/image_raw", lambda m: st.__setitem__("image", m), 2)
    track, t0, next_sample = [], time.time(), 0.0
    while time.time() - t0 < seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["pos"] is not None and time.time() - t0 >= next_sample:
            next_sample += 0.5
            track.append({"t": round(time.time() - t0, 1), "x": st["pos"][0], "y": st["pos"][1], "z": st["pos"][2],
                          "yaw": st["yaw"], "status": st["status"].split(";")[0], "target": st["target"]})
    OUT.mkdir(exist_ok=True)
    (OUT / "sim_watch.json").write_text(json.dumps(track))
    if st["image"] is not None:
        im = st["image"]
        a = np.frombuffer(bytes(im.data), np.uint8).reshape(im.height, im.width, 4)[:, :, 2::-1]
        np.save(OUT / "sim_watch_camera.npy", a)
    print(f"recorded {len(track)} samples over {seconds:.0f} s -> rl/runs/sim_watch.json", flush=True)


if __name__ == "__main__":
    main()
