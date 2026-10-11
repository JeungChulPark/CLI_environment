"""quick code-path test of live_cores.py on a YCB-V image (real objects), orig + hybrid"""
import os, sys, yaml, cv2, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
cfg = yaml.safe_load(open(os.path.expanduser("~/DeepLearning/CLI_environment/objpose/output/live_260901_cbnu_bigeightcircle_orbslam3_f2000_dup5/sam6d_config.yaml")))
os.environ["OBJPOSE_LIVE_MODE"] = sys.argv[1]; os.environ["OBJPOSE_ORIG_DESC"] = sys.argv[2]; os.environ["OBJPOSE_ORIG_THRESH"] = "0.3"
import live_cores
c = live_cores.LiveCore(cfg)
d = os.path.expanduser("~/DeepLearning/Dataset/bop/ycbv/test/000048")
bgr = cv2.imread(d + "/rgb/000001.png"); dep = cv2.imread(d + "/depth/000001.png", -1).astype(np.float32) * 0.1
K = np.array([[1066.778, 0, 312.987], [0, 1067.487, 241.311], [0, 0, 1]])
rows, ms, nb, _ = c.process(bgr, dep, K)
print("OK", sys.argv[1], [r["object"] for r in rows], ms)
