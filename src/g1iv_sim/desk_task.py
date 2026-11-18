"""Project task: official G1-29DoF + Inspire WHOLEBODY task with a white desk, black floor, no walls.

Registered id: G1IV-Desk-Cylinder-G129-Inspire-Wholebody   ("Wholebody" in the id is required: the
official sim_main.py switches to the locomotion-policy action provider based on that substring).

Everything that is not scene layout is inherited unchanged from the official
MoveCylinderG129InspireWholebodyEnvCfg (robot articulation, actuators, sim dt=0.005, decimation=4,
observation terms that feed DDS/teleimager, camera presets, events).
Scene values come from configs/scene_desk.yaml (env G1IV_SCENE overrides the path,
G1IV_SCENE_OVERRIDES='["desk.width=1.2", ...]' overrides single keys).
Policy cameras come from configs/camera_rig.yaml (Unitree G1+Inspire data-collection rig: head stereo + palm-side
wrist cameras, docs/CAMERA_RIG.md); G1IV_RIG / G1IV_RIG_OVERRIDES work the same way. use_rig: false restores
the official presets.
"""
from __future__ import annotations

import json
import math
import os
from collections.abc import Callable

import gymnasium as gym
import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
import numpy as np
import torch
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.utils import configclass

from tasks.common_config import G1RobotPresets  # official
from tasks.common_event.event_manager import SimpleEvent
from tasks.g1_tasks.move_cylinder_g1_29dof_inspire_wholebody.move_cylinder_g1_29dof_inspire_hw_env_cfg import (  # official
    MoveCylinderG129InspireWholebodyEnvCfg,
    ObjectTableSceneCfg,
)

from g1iv_common import hand_model as HM
from g1iv_common import hands as _hands
from g1iv_common.config import load_config
from g1iv_common.config import ws_path
from g1iv_sim.camera_rig import apply_rig_hardware, build_rig, pinhole_spawn_params

TASK_ID = "G1IV-Desk-Cylinder-G129-Inspire-Wholebody"
HAND_LINK_EXPRS = ("L_.*", "R_.*", "left_hand_base_link", "right_hand_base_link")


def scene_config() -> dict:
    ov = json.loads(os.environ.get("G1IV_SCENE_OVERRIDES", "[]") or "[]")
    return load_config(os.environ.get("G1IV_SCENE", "configs/scene_desk.yaml"), ov)


SCN = scene_config()


def rig_config() -> dict:
    ov = json.loads(os.environ.get("G1IV_RIG_OVERRIDES", "[]") or "[]")
    return load_config(os.environ.get("G1IV_RIG", "configs/camera_rig.yaml"), ov)


RIG = rig_config()


def yaw_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    h = math.radians(yaw_deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))  # (w, x, y, z)


def rpy_quat(r_deg: float, p_deg: float, y_deg: float) -> tuple[float, float, float, float]:
    r, p, y = (math.radians(v) / 2.0 for v in (r_deg, p_deg, y_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def look_at_quat_world(pos, target) -> tuple[float, float, float, float]:
    """Quaternion (w,x,y,z) for a camera in Isaac Lab 'world' convention (+X forward, +Z up)."""
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    l = np.cross([0.0, 0.0, 1.0], f)
    l /= np.linalg.norm(l)
    u = np.cross(f, l)
    R = np.column_stack([f, l, u])
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return (w, x, y, z)


def _spawn_preview_surface_no_specular(prim_path: str, cfg):
    """UsdPreviewSurface with the specular workflow and a black specular colour: no sky reflection at grazing
    angles, so a 2 % albedo floor renders black instead of grey (PreviewSurfaceCfg has no specular field)."""
    from isaaclab.sim.spawners.materials import visual_materials as _vm
    from pxr import Gf, Sdf

    shader = _vm.spawn_preview_surface(prim_path, cfg)
    shader.CreateAttribute("inputs:useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    shader.CreateAttribute("inputs:specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    return shader


@configclass
class NoSpecularPreviewSurfaceCfg(sim_utils.PreviewSurfaceCfg):
    func: Callable = _spawn_preview_surface_no_specular


def _matte(color, roughness, metallic, specular=True):
    cls = sim_utils.PreviewSurfaceCfg if specular else NoSpecularPreviewSurfaceCfg
    return cls(diffuse_color=tuple(color), roughness=float(roughness), metallic=float(metallic))


def _static_box(prim, size, pos, color, rough, metal, mu_s, mu_d, yaw_deg=0.0, specular=True):
    return AssetBaseCfg(
        prim_path=prim,
        init_state=AssetBaseCfg.InitialStateCfg(pos=tuple(pos), rot=yaw_quat(yaw_deg)),
        spawn=sim_utils.CuboidCfg(
            size=tuple(size),
            collision_props=sim_utils.CollisionPropertiesCfg(),   # static collider (no rigid body)
            visual_material=_matte(color, rough, metal, specular),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=float(mu_s), dynamic_friction=float(mu_d), restitution=0.0,
                friction_combine_mode="max", restitution_combine_mode="min"),
        ),
    )


def desk_parts(d: dict) -> dict[str, AssetBaseCfg]:
    """Top + 4 legs. Height is floor -> UPPER face of the top (top_surface_height)."""
    W, D, H, T = d["width"], d["depth"], d["top_surface_height"], d["top_thickness"]
    L, inset = d["leg_size"], d["leg_inset"]
    cx, cy = d["center_xy"]
    yaw = math.radians(d.get("yaw_deg", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    args = (d["color"], d["roughness"], d["metallic"], d["static_friction"], d["dynamic_friction"])
    # Flat prim names directly under env_.*: Isaac Lab spawners need an existing parent for regex paths.
    parts = {"desk_top": _static_box("/World/envs/env_.*/G1IV_DeskTop", (D, W, T), (cx, cy, H - T / 2.0),
                                     *args, yaw_deg=d.get("yaw_deg", 0.0))}
    leg_h = H - T
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        lx = sx * (D / 2.0 - inset - L / 2.0)
        ly = sy * (W / 2.0 - inset - L / 2.0)
        px, py = cx + c * lx - s * ly, cy + s * lx + c * ly
        parts[f"desk_leg{i}"] = _static_box(f"/World/envs/env_.*/G1IV_DeskLeg{i}", (L, L, leg_h),
                                            (px, py, leg_h / 2.0), *args, yaw_deg=d.get("yaw_deg", 0.0))
    return parts


_d, _f, _o, _r, _l, _c = SCN["desk"], SCN["floor"], SCN["object"], SCN["robot"], SCN["lights"], SCN["cameras"]
_robot_cfg = G1RobotPresets.g1_29dof_inspire_wholebody(init_pos=tuple(_r["init_pos"]), init_rot=yaw_quat(_r["init_yaw_deg"]))
# Hand model (docs/HAND_RH56E2.md): body = official G1 29-DoF rev 1.0 (the G1 EDU Plus body) in both cases; rh56e2
# swaps in the robot USD with Inspire RH56E2 hands (same link/joint names and order as the official asset).
HAND = HM.from_scene(SCN)
_hands.set_ranges(HM.channel_ranges(HAND))
if HAND["usd"]:
    if not os.path.isfile(HAND["usd"]):
        raise FileNotFoundError(f"[g1iv] RH56E2 robot USD missing: {HAND['usd']} -- build it (docs/HAND_RH56E2.md: "
                                "tools/cad/make_inspire_e2_hand.py, then tools/build_inspire_e2_usd.py) or run with "
                                "--scene-set robot.hand_model=inspire_dfx_official")
    _robot_cfg.spawn.usd_path = HAND["usd"]
print(f"[g1iv] robot USD: {_robot_cfg.spawn.usd_path} (hand model {HAND['name']})")
if _r.get("debug_fix_base", False):
    _robot_cfg.spawn.articulation_props.fix_root_link = True  # DEBUG ONLY (see configs/scene_desk.yaml)
RIG_HW_LOG: dict = {}
if RIG.get("use_rig", True):
    _official_robot_spawn = _robot_cfg.spawn.func

    def _spawn_robot_with_camera_hardware(prim_path, cfg, *args, **kwargs):
        """Official USD spawn + camera hardware of the rig (before physics starts; visuals only, and the official
        asset's tall camera-base bracket removed). See camera_rig.apply_rig_hardware."""
        prim = _official_robot_spawn(prim_path, cfg, *args, **kwargs)
        for root in sim_utils.find_matching_prims(prim_path):
            RIG_HW_LOG[str(root.GetPath())] = apply_rig_hardware(root, RIG, ws_path("assets/camera_rig"))
        print(f"[g1iv] camera hardware: {json.dumps(RIG_HW_LOG)[:400]}")
        return prim

    _robot_cfg.spawn.func = _spawn_robot_with_camera_hardware

_obj_z = _d["top_surface_height"] + _o["height"] / 2.0 + 0.002
_DESK = desk_parts(_d)
_observer = _c.get("observer", {})


def _cylinder_cfg(prim_name: str, xy, color) -> RigidObjectCfg:
    """Graspable cylinder on the desk (shape, mass and friction of object: in scene_desk.yaml)."""
    return RigidObjectCfg(
        prim_path=f"/World/envs/env_.*/{prim_name}",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(float(xy[0]), float(xy[1]), _obj_z), rot=(1, 0, 0, 0)),
        spawn=sim_utils.CylinderCfg(
            radius=float(_o["radius"]), height=float(_o["height"]), axis="Z",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=float(_o["mass"])),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            activate_contact_sensors=True,
            visual_material=_matte(color, 0.6, 0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=float(_o["static_friction"]), dynamic_friction=float(_o["dynamic_friction"]),
                restitution=float(_o["restitution"]), friction_combine_mode="max", restitution_combine_mode="min"),
        ),
    )


def _contact_cfg(prim_name: str) -> ContactSensorCfg:
    """Hand <-> cylinder contact (evaluation label only; never a policy observation)."""
    return ContactSensorCfg(
        prim_path=f"/World/envs/env_.*/{prim_name}",
        filter_prim_paths_expr=[f"/World/envs/env_.*/Robot/{e}" for e in HAND_LINK_EXPRS],
        history_length=1, update_period=0.0, debug_vis=False)


def object_entities(scn: dict) -> dict[str, str]:
    """Colour name -> scene entity key of every graspable cylinder (the first one is the default target)."""
    ex = scn.get("extra_objects") or {}
    out = {str(ex.get("primary_name", "red")): "object"}
    if ex.get("enabled", False):
        items = list(ex.get("items") or [])
        if len(items) > 2:
            raise ValueError("extra_objects.items: at most 2 (scene entities object_1, object_2)")
        for i, it in enumerate(items, 1):
            if it["name"] in out:
                raise ValueError(f"extra_objects: duplicate name {it['name']!r}")
            out[str(it["name"])] = f"object_{i}"
    return out


_EXTRA = list((SCN.get("extra_objects") or {}).get("items") or []) if (SCN.get("extra_objects") or {}).get("enabled") else []
OBJECT_ENTITIES = object_entities(SCN)


@configclass
class G1IVDeskSceneCfg(ObjectTableSceneCfg):
    # ---- removed official props: walls/ceiling (warehouse USD) and both packing tables ----
    room_walls = None
    packing_table1 = None
    packing_table2 = None
    robot_camera = None          # official third-person robot camera (not a policy input here)
    front_right_camera: CameraCfg | None = None   # head stereo right eye (camera rig), set in __post_init__

    robot = _robot_cfg

    # ---- white desk (static colliders): top + 4 legs ----
    desk_top = _DESK["desk_top"]
    desk_leg0 = _DESK["desk_leg0"]
    desk_leg1 = _DESK["desk_leg1"]
    desk_leg2 = _DESK["desk_leg2"]
    desk_leg3 = _DESK["desk_leg3"]

    floor = _static_box("/World/G1IV_Floor", (_f["size_xy"][0], _f["size_xy"][1], _f["thickness"]),
                        (0.0, 0.0, -_f["thickness"] / 2.0), _f["color"], _f["roughness"], _f["metallic"],
                        _f["static_friction"], _f["dynamic_friction"],
                        specular=bool(_f.get("specular_reflection", False)))

    object = _cylinder_cfg("Object", _o["xy"], _o["color"])
    # hand <-> object contact (evaluation label only; never a policy observation)
    object_contact = _contact_cfg("Object")
    # extra cylinders (extra_objects: in scene_desk.yaml), fixed positions, other colours
    object_1 = _cylinder_cfg("Object1", _EXTRA[0]["xy"], _EXTRA[0]["color"]) if len(_EXTRA) > 0 else None
    object_1_contact = _contact_cfg("Object1") if len(_EXTRA) > 0 else None
    object_2 = _cylinder_cfg("Object2", _EXTRA[1]["xy"], _EXTRA[1]["color"]) if len(_EXTRA) > 1 else None
    object_2_contact = _contact_cfg("Object2") if len(_EXTRA) > 1 else None

    light = AssetBaseCfg(prim_path="/World/light",
                         spawn=sim_utils.DomeLightCfg(color=tuple(_l["dome"]["color"]), intensity=float(_l["dome"]["intensity"])))
    key_light = AssetBaseCfg(prim_path="/World/G1IV_KeyLight",
                             init_state=AssetBaseCfg.InitialStateCfg(rot=rpy_quat(*_l["distant"]["rpy_deg"])),
                             spawn=sim_utils.DistantLightCfg(color=tuple(_l["distant"]["color"]),
                                                             intensity=float(_l["distant"]["intensity"]),
                                                             angle=float(_l["distant"]["angle_deg"])))

    # external observer camera (viewport/preview only). Name kept = official "world_camera", which
    # sim_main.py excludes from per-step updates by default (--camera_exclude world_camera).
    world_camera = CameraCfg(
        prim_path="/World/PerspectiveCamera",
        update_period=0.5, height=480, width=640, data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=12.0, horizontal_aperture=20.0, clipping_range=(0.05, 100.0)),
        offset=CameraCfg.OffsetCfg(pos=tuple(_observer.get("pos", (-0.6, -1.6, 1.6))),
                                   rot=look_at_quat_world(_observer.get("pos", (-0.6, -1.6, 1.6)),
                                                          _observer.get("look_at", (1.0, 0.0, 0.7))),
                                   convention="world"),
    ) if _observer.get("enabled", True) and _observer.get("sensor", True) else None


def rig_camera_cfgs(rig_cfg: dict, update_period: float) -> dict[str, CameraCfg]:
    """Scene camera configs of the camera rig: optical centre / orientation in the parent link (ROS optical
    frame), calibrated focal length / principal point. Lens distortion is authored on the prims after the
    env exists (g1iv_sim.ext, needs the omni_lens_distortion schema enabled at app start)."""
    out = {}
    for c in build_rig(rig_cfg):
        sp = pinhole_spawn_params(c.render)
        out[c.name] = CameraCfg(
            prim_path=f"/World/envs/env_.*/Robot/{c.parent_link}/{c.name}",
            update_period=update_period, height=c.intr.height, width=c.intr.width, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(clipping_range=tuple(c.clipping), focus_distance=400.0, **sp),
            offset=CameraCfg.OffsetCfg(pos=tuple(float(v) for v in c.pos), rot=c.quat_wxyz(), convention="ros"))
    return out


def _apply_camera_cfg(scene):
    if RIG.get("use_rig", True):
        for name, cfg in rig_camera_cfgs(RIG, float(_c["update_period"])).items():
            setattr(scene, name, cfg)
    for name in ("front_camera", "front_right_camera", "left_wrist_camera", "right_wrist_camera"):
        cam = getattr(scene, name, None)
        if cam is None:
            continue
        if not RIG.get("use_rig", True):
            cam.width, cam.height = int(_c["width"]), int(_c["height"])
        cam.update_period = float(_c["update_period"])
        if name == "front_camera" and _c.get("depth", False):
            cam.data_types = ["rgb", "distance_to_image_plane"]
        # cameras not listed in cameras.policy are not created at all (no render product, no render cost);
        # the official --camera_exclude only stops their read-back while the renderer keeps drawing them
        if name not in _c.get("policy", []):
            setattr(scene, name, None)


@configclass
class G1IVDeskEnvCfg(MoveCylinderG129InspireWholebodyEnvCfg):
    scene: G1IVDeskSceneCfg = G1IVDeskSceneCfg(num_envs=1, env_spacing=2.5, replicate_physics=True)

    def __post_init__(self):
        super().__post_init__()  # official sim dt / decimation / physx / materials / events
        _apply_camera_cfg(self.scene)
        if not _c.get("policy") or not _c.get("image_server_stream", True):
            # no policy cameras, or demo mode: drop the official camera observation term (reads every camera every
            # step and feeds the official image server / PICO stream; without cameras it would fall back to the
            # observer camera or print "no camera images" every step)
            self.observations.policy.camera_image = None
        self.scene.object_contact.update_period = self.sim.dt
        for k in ("object_1_contact", "object_2_contact"):
            if getattr(self.scene, k, None) is not None:
                getattr(self.scene, k).update_period = self.sim.dt
        if _observer.get("enabled", True):
            self.viewer.eye = tuple(_observer["pos"])
            self.viewer.lookat = tuple(_observer["look_at"])
        # official "reset_object_self" used +/-5 cm noise; use the YAML noise (default 0) instead
        nx, ny = _o.get("reset_noise_xy", [0.0, 0.0])
        self.event_manager.register("reset_object_self", SimpleEvent(
            func=lambda env: base_mdp.reset_root_state_uniform(
                env, torch.arange(env.num_envs, device=env.device),
                pose_range={"x": [-nx, nx], "y": [-ny, ny]}, velocity_range={},
                asset_cfg=SceneEntityCfg("object"))))
        for k in OBJECT_ENTITIES.values():      # extra cylinders: back to their fixed positions on env reset
            if k != "object":
                self.event_manager.register(f"reset_{k}", SimpleEvent(
                    func=lambda env, k=k: base_mdp.reset_root_state_uniform(
                        env, torch.arange(env.num_envs, device=env.device), pose_range={}, velocity_range={},
                        asset_cfg=SceneEntityCfg(k))))


gym.register(id=TASK_ID, entry_point="isaaclab.envs:ManagerBasedRLEnv",
             kwargs={"env_cfg_entry_point": G1IVDeskEnvCfg}, disable_env_checker=True)
