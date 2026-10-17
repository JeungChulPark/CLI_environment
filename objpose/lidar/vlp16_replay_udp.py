#!/usr/bin/env python3
"""vlp16_replay_udp.py — send the raw packets of a recorded /velodyne_packets bag to UDP like a live VLP-16 (test aid).
Lets the live LiDAR lane (vlp16_publisher.py -> hdl_graph_slam -> hub) be exercised without the sensor.
    python vlp16_replay_udp.py <lidar dir with *.db3> [--host 127.0.0.1] [--port 2368] [--rate 1.0] [--loop]
"""
import argparse, glob, sqlite3, socket, struct, time

ap = argparse.ArgumentParser()
ap.add_argument("dir"); ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2368)
ap.add_argument("--rate", type=float, default=1.0); ap.add_argument("--loop", action="store_true")
a = ap.parse_args()
c = sqlite3.connect(glob.glob(a.dir + "/*.db3")[0])
tid = {n: i for i, n in c.execute("select id,name from topics")}["/velodyne_packets"]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    t0 = None; w0 = time.monotonic(); n = 0
    for (blob,) in c.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
        o = 4 + 8; L = struct.unpack_from("<I", blob, o)[0]; o += 4 + L; o = (o + 3) & ~3
        k = struct.unpack_from("<I", blob, o)[0]; o += 4
        for _ in range(k):
            o = (o + 3) & ~3; sec, nsec = struct.unpack_from("<iI", blob, o); o += 8
            t = sec + nsec * 1e-9
            t0 = t if t0 is None else t0
            due = w0 + (t - t0) / a.rate
            d = due - time.monotonic()
            if d > 0: time.sleep(d)
            sock.sendto(blob[o:o + 1206], (a.host, a.port)); o += 1206; n += 1
    print(f"[replay] sent {n} packets", flush=True)
    if not a.loop: break
