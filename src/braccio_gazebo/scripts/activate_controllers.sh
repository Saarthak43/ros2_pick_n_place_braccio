#!/bin/bash
source /opt/ros/jazzy/setup.bash
# Source the workspace setup if it exists next to this script's install dir
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_INSTALL="$(realpath "$SCRIPT_DIR/../../../../install/setup.bash" 2>/dev/null)"
if [ -f "$WORKSPACE_INSTALL" ]; then
    source "$WORKSPACE_INSTALL"
else
    echo "Warning: Could not find workspace install/setup.bash. Controllers may not be on PATH."
fi

echo "Waiting for arm_controller to be loaded..."
for i in $(seq 1 60); do
    STATE=$(ros2 control list_controllers 2>/dev/null | grep arm_controller | awk '{print $3}')
    echo "  t=${i}s: arm_controller state = '$STATE'"
    if [ "$STATE" = "inactive" ] || [ "$STATE" = "unconfigured" ]; then
        break
    fi
    sleep 1
done

echo "Configuring and activating all controllers..."
ros2 control set_controller_state joint_state_broadcaster configured 2>/dev/null
ros2 control set_controller_state arm_controller configured 2>/dev/null  
ros2 control set_controller_state gripper_controller configured 2>/dev/null
sleep 2
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state arm_controller active
ros2 control set_controller_state gripper_controller active
echo "Final state:"
ros2 control list_controllers