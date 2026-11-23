---
library_name: lerobot
tags: [act, robotics, unitree-g1, inspire-hand, isaac-sim]
datasets: [jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2]
---
# ACT, Unitree G1 + Inspire RH56E2: walk to the desk and grasp (v2, simulation)

LeRobot ACT checkpoints (LeRobot 0.6.1) trained on the 55 scripted simulator episodes of
[jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2)
(converted to LeRobot v3.0; the locomotion loss weighted x3 through the stats).

- Inputs: 4 cameras 640x480 (head stereo `cam_left_high`/`cam_right_high`, wrists `cam_left_wrist`/`cam_right_wrist`),
  state 26 (arm 14 + hand 12). Output action 29 = arm 14 + hand 12 joint targets + locomotion command vx, vy, wz for the
  official Unitree locomotion policy (height fixed at 0.8 when executing).
- Training: batch 12, 100k steps, chunk 50, lr 1.5e-5 (backbone too), AMP, image augmentation; 49 train / 6 eval episodes.
- Layout: `checkpoints/<step>/{pretrained_model,training_state}` every 10k steps, with a git tag per step.
- Adopted: **030000** executed with `n_action_steps = 25`: 17/25 closed-loop successes in our Isaac Sim 5.1 desk scene
  (robot start 1.1-1.8 m from the stand pose). Most failures: a few centimetres of sideways stance error.
- Known limitation: the training data had the cylinder only at y -0.30..-0.10 m and starts within +/-0.3 m sideways;
  outside that range the policy walks to the memorised stand pose (overfitting check 2026-11-23).

Simulation only. Not tested on hardware.
