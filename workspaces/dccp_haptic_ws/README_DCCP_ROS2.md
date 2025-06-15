# DCCP Haptic Teleoperation for Surgical Robotics

This package provides DCCP-based real-time haptic teleoperation for surgical robotic systems using ROS2.

## Prerequisites

1. Enable DCCP kernel modules:
```bash
sudo modprobe dccp
sudo modprobe dccp_ipv4
echo dccp | sudo tee -a /etc/modules
echo dccp_ipv4 | sudo tee -a /etc/modules
```

2. Verify DCCP support:
```bash
lsmod | grep dccp
ls /proc/sys/net/dccp/
```

3. Install ROS2 dependencies:
```bash
sudo apt-get install ros-$ROS_DISTRO-omni-msgs
```

## Building

```bash
cd ~/dccp_haptic_ws
colcon build --packages-select dccp_haptic
source install/setup.bash
```

## Running with ros2 run

### Terminal 1 - Start the server:
```bash
cd ~/dccp_haptic_ws
source install/setup.bash
ros2 run dccp_haptic dccp_haptic_server
```

### Terminal 2 - Start the client:
```bash
cd ~/dccp_haptic_ws
source install/setup.bash
ros2 run dccp_haptic dccp_haptic_client localhost
```

## Running with launch files

### Run both server and client:
```bash
ros2 launch dccp_haptic dccp_haptic_system.launch.py
```

### Run server only:
```bash
ros2 launch dccp_haptic dccp_server.launch.py
```

### Run client only (connecting to remote server):
```bash
ros2 launch dccp_haptic dccp_client.launch.py server_host:=192.168.1.100
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
ros2 run dccp_haptic dccp_haptic_server --verbose
ros2 run dccp_haptic dccp_haptic_client localhost --verbose
```

### Run with custom stats interval:
```bash
ros2 run dccp_haptic dccp_haptic_server --stats-interval=5000
ros2 run dccp_haptic dccp_haptic_client localhost --stats-interval=5000
```

### Run client with custom rate:
```bash
ros2 run dccp_haptic dccp_haptic_client localhost --rate=500
```

### Connect to remote server:
```bash
ros2 run dccp_haptic dccp_haptic_client 192.168.1.100
```

## Troubleshooting

1. **DCCP module not loaded:**
   ```bash
   sudo modprobe dccp
   sudo modprobe dccp_ipv4
   lsmod | grep dccp
   ```

2. **DCCP not supported in kernel:**
   Some kernels may not have DCCP support compiled in. Check with:
   ```bash
   grep CONFIG_IP_DCCP /boot/config-$(uname -r)
   ```

3. **Permission denied errors:**
   ```bash
   # Check if DCCP is blocked by firewall
   sudo iptables -L | grep dccp
   
   # Allow DCCP traffic
   sudo iptables -A INPUT -p dccp --dport 4433 -j ACCEPT
   sudo iptables -A OUTPUT -p dccp --sport 4433 -j ACCEPT
   ```

4. **Connection refused:**
   - Ensure server is running before starting client
   - Check that port 4433 is not in use: `netstat -an | grep 4433`
   - Verify server IP address is correct

5. **DCCP service code error:**
   DCCP requires service codes. The default is 42. If you see service code errors:
   ```bash
   # Check DCCP settings
   cat /proc/sys/net/dccp/default/*
   ```

6. **ROS2 topics not found:**
   - Ensure Geomagic Touch device is running and publishing to `/phantom/state`
   - Check topics: `ros2 topic list`
   - Echo haptic data: `ros2 topic echo /phantom/state`

## Performance Monitoring

Monitor DCCP connections:
```bash
# Watch DCCP sockets
watch -n 1 'ss -dccp'

# Monitor DCCP statistics
watch -n 1 'netstat -s | grep -i dccp'
```

## Network Configuration for Surgical Applications

For optimal performance in surgical applications:

1. DCCP provides congestion control without full reliability
2. Suitable for applications that can tolerate some packet loss
3. Lower overhead than TCP but more than UDP
4. Consider network QoS for DCCP traffic

## DCCP Congestion Control

DCCP supports multiple congestion control IDs (CCIDs):
- CCID 2: TCP-like congestion control
- CCID 3: TCP-Friendly Rate Control (TFRC)

The implementation uses the kernel's default CCID.

## Known Limitations

1. DCCP is not available on all systems (requires kernel support)
2. Limited support in networking equipment (routers, firewalls)
3. May not work through NAT without special configuration
4. Less mature than TCP/UDP implementations
