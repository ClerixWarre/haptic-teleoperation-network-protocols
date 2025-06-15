# Installation Guide

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Geomagic Touch haptic device

## Step 1: Install ROS2 Humble

Follow the official ROS2 installation guide or run:
```bash
./scripts/setup/install_dependencies.sh
```

## Step 2: Install Geomagic Touch Driver

```bash
cd ~/
git clone https://github.com/IvoD1998/Geomagic_Touch_ROS2.git
cd Geomagic_Touch_ROS2
# Follow the driver installation instructions
```

## Step 3: Build the Project

```bash
# Build all workspaces
./scripts/build/build_all_workspaces.sh

# Or build individually:
./scripts/build/build_quic.sh
./scripts/build/build_dccp.sh
./scripts/build/build_sctp.sh
```

## Step 4: Verify Installation

Test each protocol individually:
- QUIC: See workspaces/quic_haptic_ws/README_QUIC_ROS2.md
- DCCP: See workspaces/dccp_haptic_ws/README_DCCP_ROS2.md  
- SCTP: See workspaces/sctp_haptic_ws/README_SCTP_ROS2.md

## Workspace Structure

```
workspaces/
├── quic_haptic_ws/     # QUIC protocol implementation
├── dccp_haptic_ws/     # DCCP protocol implementation
└── sctp_haptic_ws/     # SCTP protocol implementation
```
