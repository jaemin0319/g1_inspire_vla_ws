---
library_name: lerobot
tags: [act, robotics, unitree-g1, inspire-hand, isaac-sim]
datasets: [jammin0319/g1_inspire_walk_wide_grasp_desk065_scripted_v3]
---
# ACT, Unitree G1 + Inspire RH56E2: walk to the desk from wide random starts and grasp (v3, simulation)

LeRobot ACT checkpoints (LeRobot 0.6.1) trained on the 100 scripted simulator episodes of
[jammin0319/g1_inspire_walk_wide_grasp_desk065_scripted_v3](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_wide_grasp_desk065_scripted_v3)
(converted to LeRobot v3.0; the locomotion loss weighted x3 through the stats).

- Inputs: 4 cameras 640x480 (head stereo `cam_left_high`/`cam_right_high`, wrists `cam_left_wrist`/`cam_right_wrist`),
  state 26 (arm 14 + hand 12). Output action 29 = arm 14 + hand 12 joint targets + locomotion command vx, vy, wz for the
  official Unitree locomotion policy (height fixed at 0.8 when executing).
- Training: batch 12, chunk 50, lr 1.5e-5 (backbone too), AMP, image augmentation. Stopped at about 172k steps on
  2026-11-24; the last saved checkpoint is 160000.
- Layout: `checkpoints/<step>/{pretrained_model,training_state}` (010000, then every 20k to 160000), a git tag per step.
- Closed-loop evaluation (Isaac Sim 5.1 desk scene, 18 random starts: robot y -1..+1 m, cylinder y -0.45..+0.45 m,
  `n_action_steps = 25`): 20k 12/18, **40k 16/18**, 60k 12/18, 80k 13/18, 100k 13/18, 120k 12/18, 140k 13/18, 160k 10/18.
- Known limitation: recorded without robot self-collisions (the hands could pass through the thighs while walking);
  superseded by v4 (hand <-> body collision on, wider walking arm pose).

Simulation only. Not tested on hardware.
