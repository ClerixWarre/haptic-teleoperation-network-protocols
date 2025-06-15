# Troubleshooting Guide

## Common Issues and Solutions

### Build Issues

#### 1. CMake Cannot Find ROS2
**Problem**: `CMake Error: Could not find ROS2`

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Then rebuild
```

#### 2. Missing Protocol Libraries
**Problem**: `error: msquic.h: No such file or directory`

**Solution**:
```bash
# For QUIC (MsQuic)
wget https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
sudo apt update
sudo apt install libmsquic

# For SCTP
sudo apt install libsctp-dev
```

#### 3. Colcon Build Fails
**Problem**: `colcon build` fails with package errors

**Solution**:
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Runtime Issues

#### 1. Geomagic Touch Not Detected
**Problem**: `Failed to initialize haptic device`

**Solution**:
```bash
# Check device permissions
sudo chmod 666 /dev/ttyUSB*
# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

#### 2. High Latency on QUIC
**Problem**: QUIC latency higher than expected

**Solution**:
- Check network conditions
- Disable CPU frequency scaling:
```bash
sudo cpupower frequency-set -g performance
```
- Ensure no background processes

#### 3. DCCP Connection Refused
**Problem**: `DCCP: Connection refused`

**Solution**:
```bash
# Check if DCCP module is loaded
sudo modprobe dccp
sudo modprobe dccp_ipv4
# Verify with
lsmod | grep dccp
```

#### 4. SCTP Association Failed
**Problem**: `SCTP: Failed to establish association`

**Solution**:
```bash
# Check SCTP support
checksctp
# If not available, install
sudo apt install lksctp-tools
```

### Performance Issues

#### 1. Inconsistent Timing
**Problem**: Jitter higher than expected

**Solution**:
- Use real-time kernel
- Set process priority:
```bash
sudo nice -n -20 ./your_program
```
- Isolate CPU cores

#### 2. Packet Loss
**Problem**: Higher packet loss than reported

**Solution**:
- Check network cable/connection
- Disable power saving:
```bash
sudo ethtool -s eth0 wol d
```
- Increase socket buffer sizes

### Debug Commands

#### Check ROS2 Topics
```bash
ros2 topic list
ros2 topic echo /omni_state
ros2 topic hz /omni_state
```

#### Monitor Network Performance
```bash
# TCP/UDP statistics
netstat -s

# Real-time network monitoring
iftop -i eth0

# Protocol-specific monitoring
ss -anp | grep -E "quic|dccp|sctp"
```

#### System Performance
```bash
# CPU frequency
watch -n 1 "cat /proc/cpuinfo | grep MHz"

# Process priority
ps -eo pid,ni,comm | grep haptic

# Memory usage
free -h
```

### Log File Locations

- Build logs: `workspaces/*/log/latest/`
- Experiment logs: `experimental_results/logs/*/`
- System logs: `/var/log/syslog`

### Getting Help

1. Check existing issues on GitHub
2. Enable debug logging:
   ```bash
   export RCUTILS_LOG_MIN_SEVERITY=DEBUG
   ```
3. Collect system information:
   ```bash
   ros2 doctor
   ```
4. Create detailed bug report with:
   - OS version
   - ROS2 version
   - Error messages
   - Steps to reproduce
