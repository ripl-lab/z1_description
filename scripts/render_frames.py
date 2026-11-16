"""Render the end of arm of each variant from four sides, with the x, y and z axes of every frame drawn and named.

Each picture is in the gripperStator frame (frame G of the lab CAD), with link05 and link06 for context. Axes are
red, green and blue for x, y and z, 22 mm long. Use the pictures to check that a part, a camera or a frame sits and
points where the CAD puts it. The arm is at its zero pose.

    python scripts/render_frames.py                          # every variant, PNG files in ./z1_frames
    python scripts/render_frames.py bota_probe --out /tmp/f  # one variant

It needs a ROS environment with this package built (xacro, ament_index) and yourdfpy, MuJoCo and Pillow. The
default pixi environment of b2_ws has all of them. MuJoCo renders offscreen through EGL.
"""

import argparse
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import xacro  # noqa: E402
import yourdfpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

VARIANTS = ("unitree_gripper", "bota_probe", "robotiq_2f85")
W, H = 1000, 760  # px, one view
AXIS = 0.022  # m, length of each drawn axis
# Frames that get a name in the picture. Every frame away from gripperStator gets axes.
NAMED = ("gripperStator", "gripperMover", "link06", "z1_tcp", "bota_wrench", "probe_ball", "probe_tip",
         "camera_link", "camera_bottom_screw_frame", "d405_link", "d405_bottom_screw_frame")
PRINTED, METAL, DARK, CAMERA = [0.85, 0.45, 0.1, 1], [0.75, 0.75, 0.8, 1], [0.2, 0.2, 0.22, 1], [0.55, 0.6, 0.7, 1]
COLOUR = {"bota_mount": PRINTED, "robotiq_mount": PRINTED, "bota_probe": PRINTED, "d405_mount": PRINTED,
          "camera_mount": PRINTED, "bota_sensor": METAL, "robotiq_2f85": DARK, "d405_link": CAMERA, "camera_link": CAMERA}
# Each view: name, azimuth and elevation (deg) of the MuJoCo free camera.
VIEWS = (("front left, from above", 215, -25), ("side, from +y", 270, -2), ("top", 270, -89),
         ("front right, from below", 145, 25))
# Where each view looks (m, frame G) and from how far (m).
FOCUS = {"unitree_gripper": ([0.07, 0, -0.01], 0.38), "bota_probe": ([0.09, 0, 0], 0.42), "robotiq_2f85": ([0.13, 0, 0], 0.5)}


def load(variant):
    """Expand the variant's URDF and return it with every link's pose in frame G (4x4, m)."""
    doc = xacro.process_file(str(Path(get_package_share_directory("z1_description")) / "urdf" / "z1.urdf.xacro"),
                             mappings={"variant": variant, "sim_ignition": "false"})
    text = doc.toxml()
    for pkg in ("z1_description", "realsense2_description"):
        text = text.replace(f"package://{pkg}/", os.path.realpath(get_package_share_directory(pkg)) + "/")
    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as f:
        f.write(text)
        f.flush()
        robot = yourdfpy.URDF.load(f.name, load_meshes=False, build_scene_graph=True)
    to_g = np.linalg.inv(robot.get_transform("gripperStator"))
    return robot, {link: to_g @ robot.get_transform(link) for link in robot.link_map}


def visual(robot, link, pose):
    """The visual meshes of one link as one mesh in frame G, or None."""
    parts = []
    for v in robot.link_map[link].visuals:
        if v.geometry.mesh is None:
            continue
        m = trimesh.load(v.geometry.mesh.filename, force="mesh")
        if v.geometry.mesh.scale is not None:
            m.apply_scale(float(np.ravel(v.geometry.mesh.scale)[0]))
        m.apply_transform(pose @ (v.origin if v.origin is not None else np.eye(4)))
        parts.append(m)
    return trimesh.util.concatenate(parts) if parts else None


def scene(robot, poses):
    """A MuJoCo model of the end of arm: one static mesh per link and three capsules per frame."""
    spec = mujoco.MjSpec()
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
    spec.visual.headlight.ambient, spec.visual.headlight.diffuse = [0.45] * 3, [0.6] * 3
    links = [l for l in poses if l not in ("world", "link00", "link01", "link02", "link03", "link04")]
    for i, link in enumerate(links):
        m = visual(robot, link, poses[link])
        if m is not None:
            spec.add_mesh(name=f"mesh{i}", uservert=m.vertices.ravel().tolist(), userface=m.faces.ravel().tolist())
            spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f"mesh{i}",
                                    rgba=COLOUR.get(link, [0.45, 0.45, 0.48, 1]))
    # The lab parts all sit at gripperStator; drawing their axes again would hide its own.
    frames = [l for l in links if l in NAMED or not np.allclose(poses[l], np.eye(4))]
    for link in frames:
        o, R = poses[link][:3, 3], poses[link][:3, :3]
        for k, rgba in enumerate(([1, 0, 0, 1], [0, 0.8, 0, 1], [0, 0.3, 1, 1])):
            spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[*o, *(o + AXIS * R[:, k])],
                                    size=[0.0009, 0, 0], rgba=rgba)
    return spec.compile(), frames


def pixel(p, cam, fovy):
    """Image position of point p (frame G) seen by a MuJoCo free camera."""
    az, el = np.radians(cam.azimuth), np.radians(cam.elevation)
    ahead = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(ahead, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, ahead)
    v = p - (np.asarray(cam.lookat) - cam.distance * ahead)
    f = (H / 2) / np.tan(np.radians(fovy) / 2)
    return W / 2 + f * (v @ right) / (v @ ahead), H / 2 - f * (v @ up) / (v @ ahead)


def render(variant, out):
    """Write <out>/<variant>.png: the four views on one sheet."""
    robot, poses = load(variant)
    model, frames = scene(robot, poses)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    font = ImageFont.load_default(size=15)
    sheet = Image.new("RGB", (2 * W, 2 * H))
    with mujoco.Renderer(model, H, W) as renderer:
        for i, (title, azimuth, elevation) in enumerate(VIEWS):
            cam = mujoco.MjvCamera()
            cam.lookat[:], cam.distance = FOCUS[variant]
            cam.azimuth, cam.elevation = azimuth, elevation
            renderer.update_scene(data, cam)
            img = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(img)
            for link in (l for l in frames if l in NAMED):
                x, y = pixel(poses[link][:3, 3], cam, model.vis.global_.fovy)
                if 0 < x < W and 0 < y < H:
                    draw.text((x + 6, y - 18), link, fill="white", font=font, stroke_width=2, stroke_fill="black")
            draw.text((10, 8), f"{variant}: {title}. x red, y green, z blue, in gripperStator", fill="yellow",
                      font=font, stroke_width=2, stroke_fill="black")
            sheet.paste(img, ((i % 2) * W, (i // 2) * H))
    path = out / f"{variant}.png"
    sheet.save(path)
    print(f"wrote {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("variants", nargs="*", metavar="VARIANT", help=f"one or more of {', '.join(VARIANTS)} (default: all)")
    p.add_argument("--out", type=Path, default=Path("z1_frames"), help="folder for the PNG files")
    args = p.parse_args()
    unknown = set(args.variants) - set(VARIANTS)
    if unknown:
        p.error(f"unknown variant {', '.join(sorted(unknown))}: use {', '.join(VARIANTS)}")
    args.variants = args.variants or list(VARIANTS)
    args.out.mkdir(parents=True, exist_ok=True)
    for variant in args.variants:
        render(variant, args.out)


if __name__ == "__main__":
    main()
