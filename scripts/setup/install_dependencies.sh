#!/bin/bash
set -e

echo "==============================================="
echo "Installing dependencies for haptic teleoperation project"
echo "==============================================="

# Check if running on Ubuntu 22.04
if ! lsb_release -d | grep -q "Ubuntu 22.04"; then
    echo "Warning: This script is designed for Ubuntu 22.04 LTS"
    echo "Your system: $(lsb_release -d)"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo -e "\n1. Updating package list..."
sudo apt update

echo -e "\n2. Installing ROS2 Humble dependencies..."
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep

echo -e "\n3. Installing protocol-specific dependencies..."

# SCTP
echo "   - Installing SCTP libraries..."
sudo apt install -y \
    libsctp-dev \
    libsctp1 \
    lksctp-tools

# DCCP
echo "   - Checking DCCP kernel module..."
sudo modprobe dccp 2>/dev/null || echo "DCCP module not available"
sudo modprobe dccp_ipv4 2>/dev/null || echo "DCCP IPv4 module not available"

# QUIC (MsQuic)
echo "   - Installing QUIC (MsQuic) library..."
if ! dpkg -l | grep -q libmsquic; then
    # Add Microsoft package repository
    wget -q https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb
    sudo dpkg -i packages-microsoft-prod.deb
    rm packages-microsoft-prod.deb
    sudo apt update
    sudo apt install -y libmsquic
else
    echo "     MsQuic already installed"
fi

echo -e "\n4. Installing development tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    libtool \
    autoconf \
    automake

echo -e "\n5. Installing analysis tools..."
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-matplotlib \
    python3-pandas \
    python3-scipy

echo -e "\n6. Installing network monitoring tools..."
sudo apt install -y \
    net-tools \
    iftop \
    nethogs \
    tcpdump \
    wireshark

echo -e "\n7. Installing performance monitoring tools..."
sudo apt install -y \
    htop \
    iotop \
    sysstat \
    linux-tools-generic

echo -e "\n8. Setting up ROS2 dependencies..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo -e "\n9. Creating workspace directories..."
mkdir -p ~/haptic_ws/src

echo -e "\n==============================================="
echo "Dependency installation complete!"
echo ""
echo "Next steps:"
echo "1. Install the Geomagic Touch driver:"
echo "   git clone https://github.com/IvoD1998/Geomagic_Touch_ROS2.git"
echo ""
echo "2. Build the project:"
echo "   ./scripts/build/build_all_workspaces.sh"
echo ""
echo "3. Check if DCCP is available:"
echo "   lsmod | grep dccp"
echo ""
echo "Note: You may need to logout and login again for all changes to take effect."
echo "==============================================="
