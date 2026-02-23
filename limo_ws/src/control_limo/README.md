# control_limo

`control_limo` contains the parking behavior stack and simulation orchestration for the LIMO platform.

## Nodes

- `go_to_pose`: Low-level pose controller (goal -> cmd_vel).
- `parking_spot_navigator`: Staged parking goal publication.
- `parking_spot_randomizer_node`: Randomizes parking occupancy in simulation.
- `parking_spot_indicator_node`: Updates visual indicators from occupancy state.

## Launch files

- `launch/tags_parking_full.launch.py`: Complete simulation orchestration (Gare scenario).
- `launch/sim_perception.launch.py`: Multi-tag perception-only simulation (No control).

## Run full parking pipeline

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_limo tags_parking_full.launch.py
```

## Useful launch arguments

```bash
ros2 launch control_limo tags_parking_full.launch.py \
  target_spot_index:=2 \
  align_to_corridor:=true \
  reach_tolerance:=0.20 \
  distance_tolerance:=0.20
```

## Build

```bash
colcon build --symlink-install --packages-select control_limo
```

## Troubleshooting

1. Confirm `/detections` is active.
2. Confirm `/tag_only_base_pose` is active.
3. Confirm navigator is publishing goals.
4. Confirm controller is publishing to `/model/limo_01/cmd_vel`.
