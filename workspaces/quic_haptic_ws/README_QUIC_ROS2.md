# QUIC Haptic Teleoperation ROS2 Package

## Overview

This ROS2 package implements a QUIC-based communication system for haptic teleoperation using the Geomagic Touch device. QUIC (Quick UDP Internet Connections) provides low-latency, reliable communication ideal for real-time haptic feedback applications.

## Performance Characteristics

- **Average Latency**: 1.198 ms
- **Jitter**: 0.036 ms (exceptional timing consistency)
- **Reliability**: 98.5% packet delivery
- **Throughput**: 1.95 Mbps

## Dependencies

- ROS2 Humble
- MsQuic library (libmsquic)
- Geomagic Touch ROS2 driver
- omni_msgs package

## Package Structure

```
quic_haptic_ws/
└── src/
    └── quic_haptic/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── quic_client_launch.py
        │   ├── quic_server_launch.py
        │   └── quic_system_launch.py
        └── src/
            ├── quic_haptic_client.cpp
            └── quic_haptic_server.cpp
```

## Building

```bash
cd quic_haptic_ws
colcon build --symlink-install
source install/setup.bash
```

## Running

### Option 1: Using Launch Files (Recommended)

```bash
# Complete system (server + client)
ros2 launch quic_haptic quic_system_launch.py

# Or run separately:
# Terminal 1 - Server only
ros2 launch quic_haptic quic_server_launch.py

# Terminal 2 - Client only
ros2 launch quic_haptic quic_client_launch.py
```

### Option 2: Running Nodes Directly

```bash
# Terminal 1 - Start server
ros2 run quic_haptic quic_haptic_server

# Terminal 2 - Start client
ros2 run quic_haptic quic_haptic_client
```

## Configuration

### Server Configuration
- **Port**: 4242
- **Certificate**: Auto-generated self-signed
- **Streams**: 3 (Emergency, High Priority, Normal)

### Client Configuration
- **Server**: localhost:4242
- **Message Rate**: 1000 Hz (matching haptic device)
- **Priority Levels**: 
  - Emergency: Force > 80%
  - High: Force > 50%
  - Normal: All other messages

## QUIC Protocol Features Used

1. **Multi-streaming**: Separate streams for different priority levels
2. **0-RTT Connection**: Fast reconnection support
3. **Built-in Encryption**: TLS 1.3 security
4. **Congestion Control**: Adaptive rate control
5. **Connection Migration**: Handles network changes

## Topics

### Subscribed Topics
- `/omni_state` (omni_msgs/OmniState): Haptic device state at 1000Hz

### Published Topics
- `/omni_state_remote` (omni_msgs/OmniState): Received haptic state

## Message Format

The QUIC implementation serializes OmniState messages into a 75-byte format:
```
[1 byte header][8 bytes timestamp][24 bytes position][24 bytes velocity][12 bytes force][6 bytes reserved]
```

## Performance Tuning

For optimal performance:

1. **CPU Frequency Scaling**:
   ```bash
   sudo cpupower frequency-set -g performance
   ```

2. **Network Interface Optimization**:
   ```bash
   sudo ethtool -C eth0 rx-usecs 0 tx-usecs 0
   ```

3. **Process Priority**:
   ```bash
   sudo nice -n -20 ros2 run quic_haptic quic_haptic_server
   ```

## Troubleshooting

### MsQuic Not Found
```bash
# Install MsQuic
wget https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
sudo apt update
sudo apt install libmsquic
```

### Certificate Issues
The server auto-generates certificates. If issues persist:
```bash
# Generate manually
openssl req -new -x509 -days 365 -nodes -out server.cert -keyout server.key
```

### High Latency
- Check CPU frequency governor
- Disable power saving features
- Ensure no competing processes

## Implementation Details

### Client Architecture
- Non-blocking QUIC connection
- Priority-based stream selection
- Automatic reconnection on failure
- Performance metrics tracking

### Server Architecture
- Multi-stream handling
- Concurrent client support
- Real-time message processing
- Statistics reporting

## Research Results

In comparative testing, QUIC demonstrated:
- **50% lower latency** than SCTP
- **Lower jitter** than both DCCP and SCTP
- **Superior reliability** compared to DCCP
- **Consistent performance** under varying network conditions

This makes QUIC the optimal choice for haptic teleoperation requiring sub-millisecond precision.

## References

- [QUIC RFC 9000](https://datatracker.ietf.org/doc/html/rfc9000)
- [MsQuic Documentation](https://github.com/microsoft/msquic)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
