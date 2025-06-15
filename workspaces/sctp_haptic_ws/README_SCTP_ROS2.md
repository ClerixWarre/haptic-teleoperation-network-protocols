# SCTP Haptic Teleoperation for Surgical Robotics

This package provides SCTP-based real-time haptic teleoperation for surgical robotic systems using ROS2.

## Prerequisites

1. Install SCTP library:
```bash
sudo apt-get update
sudo apt-get install libsctp-dev lksctp-tools
```

2. Enable SCTP kernel module:
```bash
sudo modprobe sctp
echo sctp | sudo tee -a /etc/modules
```

3. Install ROS2 dependencies:
```bash
sudo apt-get install ros-$ROS_DISTRO-omni-msgs
```

## Building

```bash
cd ~/sctp_haptic_ws
colcon build --packages-select sctp_haptic_teleoperation
source install/setup.bash
```

## Running with ros2 run

### Terminal 1 - Start the server:
```bash
cd ~/sctp_haptic_ws
source install/setup.bash
ros2 run sctp_haptic_teleoperation sctp_haptic_server
```

### Terminal 2 - Start the client:
```bash
cd ~/sctp_haptic_ws
source install/setup.bash
ros2 run sctp_haptic_teleoperation sctp_haptic_client localhost
```

## Running with launch files

### Run both server and client:
```bash
ros2 launch sctp_haptic_teleoperation sctp_haptic_system.launch.py
```

### Run server only:
```bash
ros2 launch sctp_haptic_teleoperation sctp_server.launch.py
```

### Run client only (connecting to remote server):
```bash
ros2 launch sctp_haptic_teleoperation sctp_client.launch.py server_host:=192.168.1.100
```

## Command Line Options

### Server Options:
- `--verbose` or `-v`: Enable verbose logging
- `--quiet` or `-q`: Show only warnings and errors
- `--no-color`: Disable colored output
- `--no-timestamp`: Disable timestamps in logs
- `--stats-interval=N`: Show statistics every N messages
- `--processing-delay=X`: Add artificial processing delay (ms)
- `--random-delay`: Randomize the processing delay
- `--enable-data-logging`: Enable detailed data logging
- `--enable-protocol-analysis`: Enable protocol analysis
- `--threads=N`: Set thread pool size (default: CPU cores)

### Client Options:
- First argument: Server hostname/IP (default: localhost)
- `--verbose` or `-v`: Enable verbose logging
- `--quiet` or `-q`: Show only warnings and errors
- `--no-color`: Disable colored output
- `--no-timestamp`: Disable timestamps in logs
- `--stats-interval=N`: Show statistics every N messages
- `--enable-stream-logging`: Enable stream logging
- `--enable-data-logging`: Enable detailed data logging
- `--rate=N`: Set initial message rate in Hz (200-1000)

## Examples

### Run with verbose logging:
```bash
ros2 run sctp_haptic_teleoperation sctp_haptic_server --verbose
ros2 run sctp_haptic_teleoperation sctp_haptic_client localhost --verbose
```

### Run with custom stats interval:
```bash
ros2 run sctp_haptic_teleoperation sctp_haptic_server --stats-interval=5000
ros2 run sctp_haptic_teleoperation sctp_haptic_client localhost --stats-interval=5000
```

### Run client with custom rate:
```bash
ros2 run sctp_haptic_teleoperation sctp_haptic_client localhost --rate=500
```

### Connect to remote server:
```bash
ros2 run sctp_haptic_teleoperation sctp_haptic_client 192.168.1.100
```

## Troubleshooting

1. **SCTP module not loaded:**
   ```bash
   sudo modprobe sctp
   lsmod | grep sctp
   ```

2. **Permission denied errors:**
   ```bash
   # Check if SCTP is blocked by firewall
   sudo iptables -L | grep sctp
   
   # Allow SCTP traffic
   sudo iptables -A INPUT -p sctp --dport 4433 -j ACCEPT
   sudo iptables -A OUTPUT -p sctp --sport 4433 -j ACCEPT
   ```

3. **Connection refused:**
   - Ensure server is running before starting client
   - Check that port 4433 is not in use: `netstat -an | grep 4433`
   - Verify server IP address is correct

4. **ROS2 topics not found:**
   - Ensure Geomagic Touch device is running and publishing to `/phantom/state`
   - Check topics: `ros2 topic list`
   - Echo haptic data: `ros2 topic echo /phantom/state`

## Performance Monitoring

Monitor network performance:
```bash
# Watch SCTP associations
watch -n 1 'cat /proc/net/sctp/assocs'

# Monitor SCTP endpoints
watch -n 1 'cat /proc/net/sctp/eps'
```

## Network Configuration for Surgical Applications

For optimal performance in surgical applications:

1. Disable Nagle's algorithm (already done in code via SCTP_NODELAY)
2. Configure network QoS for SCTP traffic
3. Use dedicated network for haptic control
4. Consider using multi-homing for redundancy
