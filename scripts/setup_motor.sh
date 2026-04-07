#!/bin/bash
# =============================================================
# setup_motor.sh — One-time setup for motor control inside container
# Run this ONCE after a fresh docker-compose up -d
# Usage: bash ~/scripts/setup_motor.sh
# =============================================================
set -e

echo "===== MOTOR CONTROL + LIDAR SETUP ====="

# Source ROS
source /opt/ros/humble/setup.bash

cd ~/ros2_workspaces/src

# ---- VESC + Motor Packages ----
if [ ! -d "vesc" ]; then
    echo "[1/6] Cloning vesc (ros2 branch)..."
    git clone -b ros2 https://github.com/f1tenth/vesc.git
else
    echo "[1/6] vesc already cloned, skipping."
fi

if [ ! -d "ackermann_mux" ]; then
    echo "[2/6] Cloning ackermann_mux..."
    git clone https://github.com/f1tenth/ackermann_mux.git
else
    echo "[2/6] ackermann_mux already cloned, skipping."
fi

if [ ! -d "teleop_tools" ]; then
    echo "[3/6] Cloning teleop_tools..."
    git clone https://github.com/f1tenth/teleop_tools.git
else
    echo "[3/6] teleop_tools already cloned, skipping."
fi

# ---- LiDAR Packages (Hokuyo 10LX) ----
if [ ! -d "urg_node" ]; then
    echo "[4/6] Cloning urg_node (ros2-devel)..."
    git clone -b ros2-devel https://github.com/ros-drivers/urg_node.git
else
    echo "[4/6] urg_node already cloned, skipping."
fi

if [ ! -d "urg_c" ]; then
    echo "[5/6] Cloning urg_c (ros2-devel)..."
    git clone -b ros2-devel https://github.com/ros-drivers/urg_c.git
else
    echo "[5/6] urg_c already cloned, skipping."
fi

if [ ! -d "urg_node_msgs" ]; then
    echo "[5/6] Cloning urg_node_msgs (main)..."
    git clone -b main https://github.com/ros-drivers/urg_node_msgs.git
else
    echo "[5/6] urg_node_msgs already cloned, skipping."
fi

if [ ! -d "laser_proc" ]; then
    echo "[5/6] Cloning laser_proc (ros2-devel)..."
    git clone -b ros2-devel https://github.com/ros-perception/laser_proc.git
else
    echo "[5/6] laser_proc already cloned, skipping."
fi

if [ ! -d "transport_drivers" ]; then
    echo "[6/6] Cloning transport_drivers..."
    git clone https://github.com/ros-drivers/transport_drivers.git
else
    echo "[6/6] transport_drivers already cloned, skipping."
fi

# ---- Create minimal udp_msgs if not present ----
if [ ! -d "udp_msgs" ]; then
    echo "Creating minimal udp_msgs package..."
    mkdir -p udp_msgs/msg
    cat > udp_msgs/msg/UdpPacket.msg << 'MSGEOF'
std_msgs/Header header
uint8[] data
MSGEOF
    cat > udp_msgs/package.xml << 'XMLEOF'
<?xml version="1.0"?>
<package format="3">
  <name>udp_msgs</name>
  <version>0.0.1</version>
  <description>Minimal udp_msgs</description>
  <maintainer email="dev@dev.com">dev</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
XMLEOF
    cat > udp_msgs/CMakeLists.txt << 'CMEOF'
cmake_minimum_required(VERSION 3.5)
project(udp_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UdpPacket.msg"
  DEPENDENCIES std_msgs
)
ament_package()
CMEOF
else
    echo "udp_msgs already present, skipping."
fi

if [ -d "f1tenth_system" ]; then
    echo "Cleaning empty dirs in f1tenth_system..."
    rm -rf f1tenth_system/vesc f1tenth_system/ackermann_mux f1tenth_system/teleop_tools 2>/dev/null || true
fi

# ---- Fix urg_node CMakeLists if needed ----
if [ -f "urg_node/CMakeLists.txt" ] && ! grep -q "ament_target_dependencies" urg_node/CMakeLists.txt; then
    echo "Patching urg_node CMakeLists.txt to add ament_target_dependencies..."
    sed -i '/^rclcpp_components_register_node/i \
ament_target_dependencies(urg_node PUBLIC\
  Boost\
  diagnostic_updater\
  laser_proc\
  rclcpp\
  rclcpp_components\
  sensor_msgs\
  std_srvs\
  urg_c\
  urg_node_msgs\
)' urg_node/CMakeLists.txt
fi

# Install apt deps
echo "Installing apt dependencies..."
sudo apt-get update -qq
sudo pip3 install pyusb -q

cd ~/ros2_workspaces

# ---- Build transport_drivers (io_context, serial_driver) ----
echo "Building udp_msgs..."
colcon build --packages-select udp_msgs
source install/setup.bash

echo "Building transport_drivers (io_context, serial_driver)..."
colcon build --packages-up-to serial_driver --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

# ---- Build LiDAR packages ----
echo "Building urg_c..."
colcon build --packages-select urg_c
source install/setup.bash

echo "Building urg_node_msgs..."
colcon build --packages-select urg_node_msgs
source install/setup.bash

echo "Building laser_proc..."
colcon build --packages-select laser_proc
source install/setup.bash

echo "Building urg_node..."
colcon build --packages-select urg_node
source install/setup.bash

# ---- Build VESC packages ----
echo "Building vesc_msgs..."
colcon build --packages-select vesc_msgs
source install/setup.bash

echo "Building vesc_driver vesc_ackermann vesc..."
colcon build --packages-select vesc_driver vesc_ackermann vesc
source install/setup.bash

echo "Building ackermann_mux..."
colcon build --packages-select ackermann_mux
source install/setup.bash

echo "Building teleop_tools_msgs joy_teleop..."
colcon build --packages-select teleop_tools_msgs joy_teleop
source install/setup.bash

echo "Building f1tenth_stack..."
colcon build --packages-select f1tenth_stack
source install/setup.bash

# Fix VESC port
VESC_YAML=~/ros2_workspaces/src/f1tenth_system/f1tenth_stack/config/vesc.yaml
if grep -q "/dev/sensors/vesc" "$VESC_YAML" 2>/dev/null; then
    echo "Fixing VESC port to /dev/ttyACM0..."
    sed -i 's|port: /dev/sensors/vesc|port: /dev/ttyACM0|' "$VESC_YAML"
    colcon build --packages-select f1tenth_stack
    source install/setup.bash
fi

# Fix .ros log permissions
sudo chown -R $(whoami) ~/.ros 2>/dev/null || true

echo ""
echo "===== SETUP COMPLETE ====="
echo "Now run: bash ~/scripts/start_motor.sh"
