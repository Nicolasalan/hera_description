# Hera Description ROS 2

## Building
Setup the workspace:
```bash
rm -rf build install log
source <ws>/install/setup.bash
colcon build
```
Build the package with the following command:
```bash
colcon build --packages-select my_box_bot_description
source install/setup.bash
```

## Visualization Robot
```bash
ros2 launch my_box_bot_description urdf_visualize.launch.py
```
