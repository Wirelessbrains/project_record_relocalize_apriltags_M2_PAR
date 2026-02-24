# limo_apriltag_tools/scripts

This folder contains executable wrappers installed under `lib/limo_apriltag_tools`.

Current wrappers:

- `camera_info_publisher_node`
- `camera_info_relay_node`
- `udp_camera_receiver`
- `tag_overlay_node`
- `ippe_localizer`
- `static_map_publisher`

These wrappers call Python entry points defined in `setup.py`.

For offline trajectory/map/relocalization analysis, see:

- `../../scripts/trajectory_analysis/README.md`
