# Practical Work - Simulation (Bonus Annex)

This practical work is the **simulation annex** of the project.  
It validates the same relocalization pipeline before/alongside real-robot experiments.

## Objective

Students must:

1. Launch the multi-tag simulation scenario.
2. Drive the robot manually with joystick/teleop.
3. Record a compact dataset.
4. Build offline reference artifacts.
5. Run online relocalization in RViz.

## Mandatory Requirement

Manual control is required in simulation as well.  
The operator must use joystick/teleop to generate the teach trajectory.

## Workflow

From `limo_ws/`:

```bash
source /opt/ros/humble/setup.bash
bash profiles/build_sim.sh
```

Launch multi-tag scenario:

```bash
bash profiles/launch_sim_tags_dataset.sh 7
```

Start teleop in another terminal:

```bash
bash profiles/launch_teleop_joy_sim.sh
```

Record dataset while driving:

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

Build offline reference:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

Run simulation with generated map:

```bash
bash profiles/launch_sim_tags_online_detection.sh \
  outputs/walls_tags_run_01_outputs/tag_map.yaml
```

Run online relocalization:

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose pose_stamped map xz
```

## Expected Outputs

- `outputs/walls_tags_run_01_outputs/trajetoria_camera.csv`
- `outputs/walls_tags_run_01_outputs/tag_map.yaml`
- `/relocalization/distance_m`
- `/relocalization/heading_error_deg`
- RViz relocalization markers/path

## Note on Control

`src/control_limo` is included as a **didactic support package** only (example controllers, parking demos).  
It is not the main purpose of this repository.

For students exploring trajectory-control laws, a useful reference is:

- https://github.com/AtsushiSakai/PythonRobotics/tree/master

