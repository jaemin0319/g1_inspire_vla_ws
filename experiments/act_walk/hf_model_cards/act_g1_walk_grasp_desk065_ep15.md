---
library_name: lerobot
tags: [act, robotics, unitree-g1, inspire-hand, isaac-sim]
datasets: [jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1]
---
# ACT, Unitree G1 + Inspire RH56E2: walk and grasp, first attempt (15 episodes, simulation)

LeRobot ACT checkpoints (LeRobot 0.6.1) trained on the 15 scripted simulator episodes of
[jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1)
(converted to LeRobot v3.0). Action 29 = arm 14 + hand 12 + locomotion vx, vy, wz; 4 cameras; state 26.
Batch 12, 100k steps, lr 1.5e-5, checkpoints every 20k steps (`checkpoints/<step>/`, git tag per step); 13 train / 2 eval episodes.

**Not a working policy**: in closed-loop simulation tests it always output a zero walking command (the data had more
standing than walking frames before the arm moved, so the L1-optimal command was 0) and missed the cylinder from a near
start (40k: 0/3). Kept for reference; the follow-up is `jammin0319/act_g1_walk_far_v2`.

Simulation only.
