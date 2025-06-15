#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Source the workspace based on PROTOCOL environment variable
if [ -n "$PROTOCOL" ]; then
    case $PROTOCOL in
        quic)
            source /haptic_ws/workspaces/quic_haptic_ws/install/setup.bash
            echo "QUIC workspace sourced"
            ;;
        dccp)
            source /haptic_ws/workspaces/dccp_haptic_ws/install/setup.bash
            echo "DCCP workspace sourced"
            ;;
        sctp)
            source /haptic_ws/workspaces/sctp_haptic_ws/install/setup.bash
            echo "SCTP workspace sourced"
            ;;
        *)
            echo "Unknown protocol: $PROTOCOL"
            echo "Valid options: quic, dccp, sctp"
            ;;
    esac
fi

# Execute the command passed to docker run
exec "$@"
