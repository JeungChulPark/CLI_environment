import blenderproc as bproc
"""42-view SAM-6D templates for the 21 YCB-V objects.

Same rendering as SAM-6D's Render/render_custom_templates.py (upstream version: per view
bproc.clean_up() + reload, normalize=True -> scale 1/(2*radius of 1024 surface samples),
cnos cam_poses_level0, point light 2.5x camera position / 1000 W, 50 samples, rgb/mask/xyz
saved identically). Only change: loops over several CAD files in one Blender session.

    blenderproc run render_templates_bproc.py --models <bop models dir> --out <dir> [--objs 1,2]
"""
import argparse
import os

import bpy  # noqa: F401
import cv2
import numpy as np
import trimesh

parser = argparse.ArgumentParser()
parser.add_argument("--models", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--objs", default=",".join(str(i) for i in range(1, 22)))
parser.add_argument("--cam_poses", default=os.path.join(os.environ.get(
    "PREDEF_POSES", os.path.expanduser("~/DeepLearning/Dataset/bop/ycbv_work/predefined_poses")),
    "cam_poses_level0.npy"))
args = parser.parse_args()

bproc.init()


def get_norm_info(mesh_path):
    mesh = trimesh.load(mesh_path, force="mesh")
    model_points = trimesh.sample.sample_surface(mesh, 1024)[0].astype(np.float32)
    mn, mx = np.min(model_points, axis=0), np.max(model_points, axis=0)
    radius = max(np.linalg.norm(mx), np.linalg.norm(mn))
    return 1 / (2 * radius)


for oid in [int(x) for x in args.objs.split(",")]:
    cad = os.path.join(args.models, f"obj_{oid:06d}.ply")
    save_fpath = os.path.join(args.out, f"obj_{oid:06d}", "templates")
    if os.path.isfile(os.path.join(save_fpath, "xyz_41.npy")):
        print("skip", oid); continue
    os.makedirs(save_fpath, exist_ok=True)
    scale = get_norm_info(cad)
    cam_poses = np.load(args.cam_poses)
    for idx, cam_pose in enumerate(cam_poses):
        bproc.clean_up()
        obj = bproc.loader.load_obj(cad)[0]
        obj.set_scale([scale, scale, scale])
        obj.set_cp("category_id", 1)
        cam_pose[:3, 1:3] = -cam_pose[:3, 1:3]
        cam_pose[:3, -1] = cam_pose[:3, -1] * 0.001 * 2
        bproc.camera.add_camera_pose(cam_pose)
        light_scale, light_energy = 2.5, 1000
        light1 = bproc.types.Light()
        light1.set_type("POINT")
        light1.set_location([light_scale * cam_pose[:3, -1][0], light_scale * cam_pose[:3, -1][1],
                             light_scale * cam_pose[:3, -1][2]])
        light1.set_energy(light_energy)
        bproc.renderer.set_max_amount_of_samples(50)
        data = bproc.renderer.render()
        data.update(bproc.renderer.render_nocs())
        color_bgr_0 = data["colors"][0]
        color_bgr_0[..., :3] = color_bgr_0[..., :3][..., ::-1]
        cv2.imwrite(os.path.join(save_fpath, "rgb_" + str(idx) + ".png"), color_bgr_0)
        mask_0 = data["nocs"][0][..., -1]
        cv2.imwrite(os.path.join(save_fpath, "mask_" + str(idx) + ".png"), mask_0 * 255)
        xyz_0 = 2 * (data["nocs"][0][..., :3] - 0.5)
        np.save(os.path.join(save_fpath, "xyz_" + str(idx) + ".npy"), xyz_0.astype(np.float16))
    print("done", oid, flush=True)
