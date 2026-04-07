cd ~/ros2_workspaces

colcon build --packages-select AEB_System movement_node autonomy
source install/setup.bash
ros2 launch autonomy autonomy.launch.py