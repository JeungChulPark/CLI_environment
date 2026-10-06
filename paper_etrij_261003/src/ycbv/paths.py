"""All paths of the YCB-V single-image comparison, overridable by environment variables.

Defaults match the verification machine; on another machine set the env vars (see README).
"""
import os
from pathlib import Path

HOME = Path.home()
HERE = Path(__file__).resolve().parent


def _p(env, default):
    return Path(os.environ.get(env, str(default))).expanduser()


# code
REPO = _p("OBJPOSE_REPO", HERE.parents[2])                       # CLI_environment checkout
SR = _p("SAM6D_REALTIME", REPO / "sam6d_realtime")               # our recognizer
SAM6D_ORIG = _p("SAM6D_ORIG", REPO / "sam6d_ws/sam6d_master/SAM-6D")   # original SAM-6D code
# data
YCBV = _p("YCBV_DIR", HOME / "DeepLearning/Dataset/bop/ycbv")     # BOP ycbv (models, models_eval, test, targets)
WORK = _p("YCBV_WORK", HOME / "DeepLearning/Dataset/bop/ycbv_work")  # templates, caches, assets
OUT = _p("YCBV_OUT", HERE / "out")                               # predictions, metrics
# checkpoints
FASTSAM_X = _p("FASTSAM_X", Path("/mnt/d/old/CLI_environment/sam6d_ws/sam6d_master/SAM-6D/"
                                  "Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt"))
DINOV2_VITL_DIR = _p("DINOV2_VITL_DIR", HOME / ".cache/sam6d_orig_ckpt/dinov2")  # dinov2_vitl14_pretrain.pth
PEM_CKPT = _p("PEM_CKPT", SR / "sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth")
PREDEF = _p("PREDEF_POSES", WORK / "predefined_poses")           # SAM-6D ISM utils/poses/predefined_poses

# derived (work dir layout)
TEMPLATES = WORK / "templates"            # templates/obj_0000XX/templates/{rgb,mask,xyz}_i
FEATURES = WORK / "features"              # DINOv2 cls/appe + HSV caches (ours)
CORE_REPO = WORK / "core_repo"            # stand-in REPO for Sam6DCore: assets/{clip,model_points,pem_templates}
OURS_CFG = WORK / "ycbv_objects.yaml"     # generated recognizer config
BASE_CFG = SR / "configs/yolo_ism_objects.yaml"

LABEL = os.environ.get("YCBV_RUN_LABEL", "verification run, RTX 3080 Ti Laptop")

# 21 YCB-V objects (BOP obj_id 1..21) and the paper's YOLO-World prompts
YCB_NAMES = ["002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can",
             "006_mustard_bottle", "007_tuna_fish_can", "008_pudding_box", "009_gelatin_box",
             "010_potted_meat_can", "011_banana", "019_pitcher_base", "021_bleach_cleanser",
             "024_bowl", "025_mug", "035_power_drill", "036_wood_block", "037_scissors",
             "040_large_marker", "051_large_clamp", "052_extra_large_clamp", "061_foam_brick"]
PROMPTS = ["blue coffee can", "red cracker box", "yellow sugar box", "tomato soup can",
           "yellow mustard bottle", "small tuna can", "small brown box", "small red box",
           "small blue can", "banana", "blue plastic pitcher", "white cleanser bottle", "red bowl",
           "red mug", "power drill", "wooden block", "scissors", "marker pen", "black spring clamp",
           "large black clamp", "red brick"]
CONSENSUS_OCC = 0.3       # verify.cluster_min_occupancy for YCB (paper); deployed default 0.5


def obj_name(oid):
    return f"ycbv_{int(oid):02d}"


def oid_of(name):
    return int(name.split("_")[1])
