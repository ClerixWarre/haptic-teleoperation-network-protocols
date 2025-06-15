FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # ROS2 dependencies
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Protocol libraries
    libsctp-dev \
    libsctp1 \
    lksctp-tools \
    # Network tools
    net-tools \
    iputils-ping \
    # Development tools
    gdb \
    valgrind \
    && rm -rf /var/lib/apt/lists/*

# Install MsQuic for QUIC support
RUN wget -q https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb && \
    dpkg -i packages-microsoft-prod.deb && \
    rm packages-microsoft-prod.deb && \
    apt-get update && \
    apt-get install -y libmsquic && \
    rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /haptic_ws

# Copy the entire repository
COPY . /haptic_ws/

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Build all workspaces
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /haptic_ws && \
    ./scripts/build/build_all_workspaces.sh"

# Set up entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
