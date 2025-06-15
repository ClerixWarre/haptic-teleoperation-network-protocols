# Docker Setup for Haptic Teleoperation

This directory contains Docker configuration for running the haptic teleoperation experiments in containers.

## Prerequisites

- Docker Engine 20.10+
- Docker Compose 2.0+
- Host system with DCCP kernel module (for DCCP protocol)

## Building the Image

```bash
# From the repository root
docker build -f docker/Dockerfile -t haptic-teleoperation .

# Or using docker-compose
cd docker
docker-compose build
```

## Running Experiments

### Using Docker Compose

```bash
# Start QUIC experiment
docker-compose up quic-server quic-client

# Start DCCP experiment (requires privileged mode)
docker-compose up dccp-server dccp-client

# Start SCTP experiment
docker-compose up sctp-server sctp-client

# Interactive development container
docker-compose run --rm dev
```

### Using Docker Run

```bash
# QUIC Server
docker run --rm -it \
  --network host \
  -e PROTOCOL=quic \
  -v $(pwd)/experimental_results:/haptic_ws/experimental_results \
  haptic-teleoperation \
  ros2 run quic_haptic quic_haptic_server

# QUIC Client (in another terminal)
docker run --rm -it \
  --network host \
  -e PROTOCOL=quic \
  -v $(pwd)/experimental_results:/haptic_ws/experimental_results \
  haptic-teleoperation \
  ros2 run quic_haptic quic_haptic_client
```

## Environment Variables

- `PROTOCOL`: Select workspace to source (quic, dccp, sctp)
- `ROS_DOMAIN_ID`: ROS2 domain ID for network isolation

## Notes

1. **Network Mode**: Uses `host` networking for performance and protocol compatibility
2. **Privileged Mode**: Required for DCCP due to kernel module requirements
3. **Volume Mounts**: Results are saved to host's `experimental_results` directory
4. **Geomagic Touch**: The haptic device must be connected to the host system

## Troubleshooting

### DCCP Not Working
```bash
# On host system, load DCCP modules
sudo modprobe dccp
sudo modprobe dccp_ipv4
```

### Permission Issues
```bash
# Add user to docker group
sudo usermod -aG docker $USER
# Logout and login again
```

### Build Cache Issues
```bash
# Force rebuild
docker-compose build --no-cache
```
