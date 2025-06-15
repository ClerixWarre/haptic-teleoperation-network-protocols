# Usage Guide

## Running the Haptic Teleoperation System

### 1. Start the Geomagic Touch Driver

In a new terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 launch omni_common omni_state.launch.py
```

### 2. Run Protocol-Specific Implementations

#### QUIC Protocol
```bash
# Terminal 1 - Server
cd workspaces/quic_haptic_ws
source install/setup.bash
ros2 run quic_haptic quic_haptic_server

# Terminal 2 - Client
cd workspaces/quic_haptic_ws
source install/setup.bash
ros2 run quic_haptic quic_haptic_client
```

#### DCCP Protocol
```bash
# Terminal 1 - Server
cd workspaces/dccp_haptic_ws
source install/setup.bash
ros2 run dccp_haptic dccp_haptic_server

# Terminal 2 - Client
cd workspaces/dccp_haptic_ws
source install/setup.bash
ros2 run dccp_haptic dccp_haptic_client
```

#### SCTP Protocol
```bash
# Terminal 1 - Server
cd workspaces/sctp_haptic_ws
source install/setup.bash
ros2 run sctp_haptic_teleoperation sctp_haptic_server

# Terminal 2 - Client
cd workspaces/sctp_haptic_ws
source install/setup.bash
ros2 run sctp_haptic_teleoperation sctp_haptic_client
```

### 3. Using Launch Files

For complete system startup:
```bash
# QUIC
ros2 launch quic_haptic quic_haptic_system.launch.py

# DCCP
ros2 launch dccp_haptic dccp_haptic_system.launch.py

# SCTP
ros2 launch sctp_haptic_teleoperation sctp_haptic_system.launch.py
```

### 4. Monitoring Performance

Performance metrics are automatically logged to:
- `experimental_results/logs/quic/`
- `experimental_results/logs/dccp/`
- `experimental_results/logs/sctp/`

### 5. Running Experiments

To run a complete experimental session:
```bash
./scripts/experiments/run_all_experiments.sh
```

This will:
1. Start each protocol in sequence
2. Collect performance metrics
3. Generate analysis reports
