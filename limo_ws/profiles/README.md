# Workspace Profiles

This folder provides helper scripts for two workflows:

1. **REAL (primary):** physical LIMO robot
2. **SIM (bonus):** Gazebo/Ignition validation

---

## REAL (Primary)

Build:

```bash
bash profiles/build_real.sh
```

Run perception/localization:

```bash
bash profiles/launch_real_perception.sh
```

Run minimal navigation:

```bash
bash profiles/launch_real_nav_minimal.sh
```

Run online relocalization:

```bash
bash profiles/launch_real_online_relocalization.sh <reference_csv_path> [pose_topic]
```

Example:

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_base_pose
```

---

## SIM (Bonus)

Build:

```bash
bash profiles/build_sim.sh
```

### Parking demo

```bash
bash profiles/launch_sim_parking.sh
```

### Dataset + online relocalization demo (walls scenario)

1) Launch scenario:

```bash
bash profiles/launch_sim_tags_dataset.sh 7
```

2) Start joystick teleop:

```bash
bash profiles/launch_teleop_joy_sim.sh
```

3) Record bag:

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

4) Build offline outputs:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

5) Optional transform (trajectory + map):

```bash
python3 profiles/rotate_reference_frame.py \
  --traj-in outputs/walls_tags_run_01_outputs/trajetoria_camera.csv \
  --traj-out outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  --map-in outputs/walls_tags_run_01_outputs/tag_map.yaml \
  --map-out outputs/walls_tags_run_01_outputs/tag_map_walls_xz.yaml
```

6) Launch simulation with selected YAML:

```bash
bash profiles/launch_sim_tags_online_detection.sh \
  outputs/walls_tags_run_01_outputs/tag_map_walls_xz.yaml
```

7) Start online relocalization + RViz:

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  /tag_only_pose pose_stamped map xz
```

---

## Notes

- Use `minimal` recording mode to keep bag size small.
- `launch_sim_online_relocalization.sh` default profile:
  - `pose_topic=/tag_only_pose`
  - `pose_msg_type=pose_stamped`
  - `frame_id=map`
  - `trajectory_plane=xz`
- If you do not use transformed `*_xz` outputs, keep original CSV and YAML together.
