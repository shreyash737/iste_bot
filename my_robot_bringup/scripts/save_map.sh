#!/bin/bash

# Map save script
# Usage: ./save_map.sh [map_name]

MAP_NAME=${1:-my_map}
MAP_DIR=~/ros2_ws/src/my_robot_bringup/maps

echo "Saving map as: $MAP_NAME"
echo "Location: $MAP_DIR"

# Create maps directory if it doesn't exist
mkdir -p $MAP_DIR

# Save the map
ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/$MAP_NAME --ros-args -p use_sim_time:=true

echo ""
echo "Map saved!"
echo "Files created:"
echo "  - $MAP_DIR/$MAP_NAME.pgm"
echo "  - $MAP_DIR/$MAP_NAME.yaml"