#!/bin/bash
set -e

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "==============================================="
echo "Building all ROS2 haptic teleoperation workspaces"
echo "==============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to build a workspace
build_workspace() {
    local ws_name=$1
    local ws_path="$ROOT_DIR/workspaces/$ws_name"
    
    echo -e "\n${YELLOW}=== Building $ws_name ===${NC}"
    
    if [ ! -d "$ws_path" ]; then
        echo -e "${RED}Error: Workspace $ws_path not found!${NC}"
        return 1
    fi
    
    cd "$ws_path"
    
    # Clean previous build if requested
    if [ "$CLEAN_BUILD" = true ]; then
        echo "Cleaning previous build..."
        rm -rf build install log
    fi
    
    # Build the workspace
    echo "Running colcon build..."
    if colcon build --symlink-install; then
        echo -e "${GREEN}✓ $ws_name built successfully!${NC}"
    else
        echo -e "${RED}✗ Failed to build $ws_name${NC}"
        return 1
    fi
    
    # Source the workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo "Workspace sourced."
    fi
    
    return 0
}

# Parse command line arguments
CLEAN_BUILD=false
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--clean]"
            exit 1
            ;;
    esac
done

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo "ROS2 Distribution: $ROS_DISTRO"
echo "Root Directory: $ROOT_DIR"

if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}Clean build requested - removing previous builds${NC}"
fi

# Build each workspace
SUCCESS_COUNT=0
FAIL_COUNT=0

# QUIC workspace
if build_workspace "quic_haptic_ws"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# DCCP workspace
if build_workspace "dccp_haptic_ws"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# SCTP workspace
if build_workspace "sctp_haptic_ws"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# Summary
echo -e "\n==============================================="
echo "Build Summary:"
echo -e "${GREEN}Successful builds: $SUCCESS_COUNT${NC}"
if [ $FAIL_COUNT -gt 0 ]; then
    echo -e "${RED}Failed builds: $FAIL_COUNT${NC}"
else
    echo -e "${GREEN}All workspaces built successfully!${NC}"
fi
echo "==============================================="

# Return error if any builds failed
if [ $FAIL_COUNT -gt 0 ]; then
    exit 1
fi

# Provide sourcing instructions
echo -e "\nTo use a specific workspace, source it with:"
echo "  source $ROOT_DIR/workspaces/quic_haptic_ws/install/setup.bash"
echo "  source $ROOT_DIR/workspaces/dccp_haptic_ws/install/setup.bash"
echo "  source $ROOT_DIR/workspaces/sctp_haptic_ws/install/setup.bash"
