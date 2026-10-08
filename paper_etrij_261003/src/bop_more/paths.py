"""All paths of the BOP single-image comparison (LM-O / TUD-L / IC-BIN / T-LESS), overridable
by environment variables. Generalisation of src/ycbv/paths.py: the dataset is chosen by
BOP_DATASET (lmo | tudl | icbin | tless), the object ids come from
<ds>/models_eval/models_info.json, object names are "<ds>_<id:02d>".
"""
import json
import os
from pathlib import Path

HOME = Path.home()
HERE = Path(__file__).resolve().parent


def _p(env, default):
    return Path(os.environ.get(env, str(default))).expanduser()


DS = os.environ.get("BOP_DATASET", "lmo").strip().lower()
assert DS in ("lmo", "tudl", "icbin", "tless", "hb"), f"BOP_DATASET={DS}"
# code
REPO = _p("OBJPOSE_REPO", HERE.parents[2])                       # CLI_environment checkout
SR = _p("SAM6D_REALTIME", REPO / "sam6d_realtime")               # our recognizer
SAM6D_ORIG = _p("SAM6D_ORIG", REPO / "sam6d_ws/sam6d_master/SAM-6D")   # original SAM-6D code
# data (variable names kept from the YCB-V scripts so that run_*/evaluate stay identical)
YCBV = _p("BOP_DS_DIR", HOME / "Dataset/bop" / DS)               # BOP <ds> (models, models_eval, test, targets)
WORK = _p("BOP_WORK", HOME / "Dataset/bop" / f"{DS}_work")        # templates, caches, assets
OUT = _p("BOP_OUT", HERE / "out" / DS)                            # predictions, metrics
# checkpoints
FASTSAM_X = _p("FASTSAM_X", Path("/mnt/d/old/CLI_environment/sam6d_ws/sam6d_master/SAM-6D/"
                                  "Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt"))
DINOV2_VITL_DIR = _p("DINOV2_VITL_DIR", HOME / ".cache/sam6d_orig_ckpt/dinov2")  # dinov2_vitl14_pretrain.pth
PEM_CKPT = _p("PEM_CKPT", SR / "sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth")
PREDEF = _p("PREDEF_POSES", HOME / "Dataset/bop/ycbv_work/predefined_poses")   # SAM-6D ISM utils/poses/predefined_poses

# derived (work dir layout)
TEMPLATES = WORK / "templates"            # templates/obj_0000XX/templates/{rgb,mask,xyz}_i
FEATURES = WORK / "features"              # DINOv2 cls/appe + HSV caches (ours)
CORE_REPO = WORK / "core_repo"            # stand-in REPO for Sam6DCore: assets/{clip,model_points,pem_templates}
OURS_CFG = _p("BOP_OURS_CFG", WORK / f"{DS}_objects.yaml")   # generated recognizer config
BASE_CFG = SR / "configs/yolo_ism_objects.yaml"

LABEL = os.environ.get("BOP_RUN_LABEL", f"{DS} run, RTX 4090")
TEST_DIR = "test_primesense" if DS == "tless" else "test"   # BOP test split folder (T-LESS: PrimeSense)

# object ids of the dataset (BOP obj_id), from models_eval/models_info.json
_info_path = YCBV / "models_eval" / "models_info.json"
OIDS = sorted(int(k) for k in json.load(open(_info_path))) if _info_path.exists() else []
# the YCB-V scripts index these by obj_id; here they are dicts keyed by obj_id.
# No text prompts are needed for the two compared methods (original ISM front end); the prompt is
# only a unique placeholder string per object for the recognizer config.
YCB_NAMES = {oid: f"{DS}_{oid:02d}" for oid in OIDS}
PROMPTS = {oid: f"{DS} object {oid}" for oid in OIDS}
CONSENSUS_OCC = float(os.environ.get("YCBV_CONSENSUS_OCC", 0.3))   # verify.cluster_min_occupancy, as for YCB (paper)


def obj_name(oid):
    return f"{DS}_{int(oid):02d}"


def oid_of(name):
    return int(name.split("_")[1])
