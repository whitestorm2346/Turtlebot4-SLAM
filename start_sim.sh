#!/bin/bash

# ==========================================
# Source ROS 2 and workspace
# ==========================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# Fix FastCDR symbol conflict with librealsense2
FASTCDR_LIB="/opt/ros/jazzy/lib/libfastcdr.so.2"

if [[ -n "${LD_PRELOAD:-}" ]]; then
    export LD_PRELOAD="${FASTCDR_LIB}:${LD_PRELOAD}"
else
    export LD_PRELOAD="${FASTCDR_LIB}"
fi


# ==========================================
# Arrow-key menu function
# ==========================================

select_option() {
    local prompt="$1"
    shift
    local options=("$@")

    local selected=0
    local key

    while true; do
        clear

        echo "======================================"
        echo "$prompt"
        echo "======================================"
        echo

        for i in "${!options[@]}"; do
            if [[ $i -eq $selected ]]; then
                echo " > ${options[$i]}"
            else
                echo "   ${options[$i]}"
            fi
        done

        echo
        echo "Use ↑ ↓ to select, Enter to confirm."

        IFS= read -rsn1 key

        if [[ $key == $'\x1b' ]]; then
            read -rsn2 key

            case "$key" in
                '[A')
                    ((selected--))
                    if ((selected < 0)); then
                        selected=$((${#options[@]} - 1))
                    fi
                    ;;
                '[B')
                    ((selected++))
                    if ((selected >= ${#options[@]})); then
                        selected=0
                    fi
                    ;;
            esac

        elif [[ -z $key ]]; then
            SELECTED_INDEX=$selected
            return
        fi
    done
}


# ==========================================
# Select Gazebo world
# ==========================================

WORLD_OPTIONS=(
    "Warehouse"
    "Depot"
    "Maze"
)

WORLD_VALUES=(
    "warehouse"
    "depot"
    "maze"
)

select_option \
    "Select Gazebo World" \
    "${WORLD_OPTIONS[@]}"

WORLD="${WORLD_VALUES[$SELECTED_INDEX]}"


# ==========================================
# Select Gazebo GUI
# ==========================================

GUI_OPTIONS=(
    "Enable Gazebo GUI"
    "Disable Gazebo GUI"
)

GUI_VALUES=(
    "true"
    "false"
)

select_option \
    "Start Gazebo GUI?" \
    "${GUI_OPTIONS[@]}"

GUI="${GUI_VALUES[$SELECTED_INDEX]}"


# ==========================================
# Launch
# ==========================================

clear

echo "======================================"
echo "Starting TurtleBot4 SLAM"
echo "======================================"
echo
echo "World       : $WORLD"
echo "Gazebo GUI  : $GUI"
echo "Exploration : frontier_exploration_ros2"
echo

exec ros2 launch tb4_slam_bringup full_system.launch.py \
    world:="$WORLD" \
    gui:="$GUI"