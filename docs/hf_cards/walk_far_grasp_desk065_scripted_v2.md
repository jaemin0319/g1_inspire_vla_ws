Simulated **Unitree G1 (29-DoF) with Inspire RH56E2 hands** walking 1.1–1.8 m up to a desk and picking up a red
cylinder with the right hand (walk → stop → grasp → lift). Isaac Sim 5.1 / Isaac Lab, built on
[unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab).

**This is scripted, simulator-only data, not human teleoperation.** Every episode was generated automatically by a
scripted expert that plans from simulator ground truth (object and robot pose). The ground truth was used only to
plan the motion; it is stored in the `eval.*` columns for evaluation and is not meant as a policy input.

- **Task:** "walk to the desk and pick up the red cylinder" (one task). Desk top height **0.65 m**, cylinder height 0.12 m.
- **Episodes:** 55 successful episodes, 27,931 frames (474–532 per episode), 25 fps. Only successes are included
  (cylinder lifted ≥ 5 cm, robot not fallen).
- **Long walk:** the robot starts far from the desk (world frame: x −1.0 to −0.4 m, y −0.3 to 0.3 m, yaw ±15°), so it
  walks 1.1–1.8 m to the grasp stance. Each episode is one continuous walk (80–128 frames, median 4.4 s) followed by
  standing and grasping; walking and standing frames before the arm moves are about 2.2 : 1.
  Cylinder position per episode: x 0.98–1.08 m, y −0.30 to −0.10 m.
- **Selection:** from 69 recorded episodes, 55 were kept by a stability filter: success, pelvis tilt ≤ 7°, pelvis
  height ≥ 0.745 m, roll/pitch rate ≤ 1.0 rad/s. Removed episodes include ones where the expert stopped off target,
  backed up and re-approached (pelvis tilt 8–9° while backing up), so no episode contains back-and-forth stepping.
- **Locomotion:** the official Unitree whole-body locomotion policy, driven by velocity commands that are recorded in
  `action.loco_cmd` = [vx, vy, wz, height]: the command the locomotion policy actually consumed at that step
  (captured inside the simulator where the policy reads it), so a model can drive walking by predicting it
  (`meta/modality.json` → action `base_command` = vx, vy, wz). The policy ignores `height` (always 0.8).
  When executing: the official policy resets the command to zero after every read, so re-publish the predicted
  command to `rt/run_command/cmd` at ≥ 50 Hz until the next prediction; it does not step for |vx| ≲ 0.3 m/s.
- **Arm/hand actions:** `action` (26) = left arm 7, right arm 7, left hand 6, right hand 6 joint position targets,
  applied by the simulator in the step following the observation. `action.hand_binary` = [left_close, right_close].
  Joint order is in `meta/info.json`; `meta/modality.json` gives the GR00T-style grouping.
- **Cameras (640×480, AV1):** head stereo `cam_left_high` / `cam_right_high` and wrist cameras `cam_left_wrist` /
  `cam_right_wrist` (RealSense D405 model). Intrinsics, distortion and mounting are in `meta/info.json` → `camera_params`.
- **Other columns:** `observation.base_vel` (body-frame linear/angular velocity), `observation.sim_time`,
  `eval.object_pos_w`, `eval.contact_force_total`, `eval.fallen`.

**Limitations:** simulation only; the scripted motions are smoother and more repetitive than human demonstrations.
The expert always stops at nearly the same pose relative to the cylinder (standard deviation about 1.3 cm forward,
0.6 cm sideways), so the data has almost no examples of reaching from an offset stance; in our simulator tests an ACT
policy trained on it walked to the desk reliably but most of its failures came from a few centimetres of sideways
stance error. The RH56E2 hand model is a simulation approximation.

Format: LeRobot dataset v2.1 (lerobot 0.3.3), checked with the LeRobot v2.1 loader and the Isaac-GR00T N1.7
`LeRobotEpisodeLoader`. Load with `LeRobotDataset("jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2")`.
