# gz_apriltag_env

Gazebo/Ignition worlds and models used for AprilTag-based simulation.

## Main world

- `worlds/walls_apriltag_limo.sdf`

This world is used for dataset collection and relocalization validation.

## Run directly with Ignition

```bash
cd <workspace_root>/limo_ws
export IGN_GAZEBO_RESOURCE_PATH=$PWD/src/gz_apriltag_env/models
ign gazebo -r $PWD/src/gz_apriltag_env/worlds/walls_apriltag_limo.sdf
```

## Recommended run via workspace profile

```bash
bash profiles/launch_sim_tags_dataset.sh
```
