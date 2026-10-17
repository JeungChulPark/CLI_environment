# laptop: raw VLP-16 data packets (no ROS) -> pickle.   python3 vlp_dump.py <n packets> <out.pkl>
import socket, pickle, sys, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20); s.bind(("", 2368)); s.settimeout(2)
n = int(sys.argv[1]); out = []; t0 = time.time()
while len(out) < n:
    d, _ = s.recvfrom(2048)
    if len(d) == 1206 and d[0] == 0xFF and d[1] == 0xEE: out.append(d)
pickle.dump({"t0": t0, "t1": time.time(), "packets": out}, open(sys.argv[2], "wb")); print(len(out), "packets", round(time.time() - t0, 2), "s")
