# Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository contains the complete implementation and experimental results for the MSc thesis: **"Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control"** by Warre Clerix at UHasselt & KULeuven.

## Overview

This research evaluates three emerging network protocols (QUIC, DCCP, SCTP) for haptic teleoperation systems using a ROS2-based experimental platform with a Geomagic Touch haptic interface. The study focuses on real-time communication requirements for surgical robotics and precision control applications.

## Key Findings

| Protocol | Avg Latency | Jitter | Reliability | Throughput | Best For |
|----------|-------------|--------|-------------|------------|----------|
| **QUIC** | 1.198 ms | 0.036 ms | 98.5% | 1.95 Mbps | **Haptic Control** |
| **DCCP** | 2.45 ms | 0.095 ms | 86.6% | 1.68 Mbps | Non-critical telemetry |
| **SCTP** | 5.231 ms | 0.683 ms | 97.6% | 1.93 Mbps | Industrial automation |

**Key Conclusions:**
- **QUIC** demonstrated superior performance for time-critical haptic control applications
- **DCCP** suffered from significant reliability issues (13.4% packet loss)
- **SCTP** provided excellent reliability but with higher latency unsuitable for sub-millisecond control

## Repository Structure

```
├── src/
│   ├── quic/
│   │   ├── client/quic_haptic_client.cpp
│   │   └── server/quic_haptic_server.cpp
│   ├── dccp/
│   │   ├── client/dccp_haptic_client.cpp
│   │   └── server/dccp_haptic_server.cpp
│   └── sctp/
│       ├── client/sctp_haptic_client.cpp
│       └── server/sctp_haptic_server.cpp
├── experimental-results/
│   ├── quic/
│   │   ├── quic_client_output.txt
│   │   └── quic_server_output.txt
│   ├── dccp/
│   │   ├── dccp_client_output.txt
│   │   └── dccp_server_output.txt
│   └── sctp/
│       ├── sctp_client_output.txt
│       └── sctp_server_output.txt
├── scripts/
│   └── build_all.sh
└── README.md
```

## Dependencies

### External Repositories
- **Geomagic Touch ROS2 Driver**: [IvoD1998/Geomagic_Touch_ROS2](https://github.com/IvoD1998/Geomagic_Touch_ROS2)
  - Provides ROS2 interface for Geomagic Touch haptic device
  - Converts haptic state to `omni_msgs/OmniState` messages
  - Essential for haptic input processing in all protocol implementations

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble distribution
- **Hardware**: Geomagic Touch haptic device (1000Hz update rate)
- **Network**: Controlled Gigabit Ethernet environment
- **Compilers**: GCC with C++17 support

### Protocol-Specific Dependencies
- **QUIC**: MsQuic library (`-lmsquic`)
- **DCCP**: Linux kernel DCCP support (built-in)
- **SCTP**: SCTP libraries (`-lsctp`)

## Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/ClerixWarre/haptic-teleoperation-network-protocols.git
cd haptic-teleoperation-network-protocols
```

### 2. Install Dependencies
```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rmw-cyclonedx

# Install protocol dependencies
sudo apt install libsctp-dev  # For SCTP
# MsQuic installation varies by system

# Set up ROS2 workspace with Geomagic Touch driver
mkdir -p ~/haptic_ws/src
cd ~/haptic_ws/src
git clone https://github.com/IvoD1998/Geomagic_Touch_ROS2.git
cd ~/haptic_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 3. Build All Protocols
```bash
chmod +x scripts/build_all.sh
./scripts/build_all.sh
```

## Running Experiments

### Terminal 1: Start Haptic Driver
```bash
source ~/haptic_ws/install/setup.bash
ros2 run geomagic_touch geomagic_node
```

### Terminal 2: Start Protocol Server
```bash
# For QUIC
cd src/quic/server
./quic_haptic_server

# For DCCP  
cd src/dccp/server
./dccp_haptic_server

# For SCTP
cd src/sctp/server
./sctp_haptic_server
```

### Terminal 3: Start Protocol Client
```bash
# For QUIC
cd src/quic/client
./quic_haptic_client

# For DCCP
cd src/dccp/client
./dccp_haptic_client

# For SCTP
cd src/sctp/client
./sctp_haptic_client
```

## Experimental Results

### Performance Metrics Summary

The [`experimental-results/`](./experimental-results/) directory contains comprehensive logs for all tested protocols:

#### QUIC Protocol Results
- **Average Latency**: 1.198 ms (min: 1.023 ms, max: 1.687 ms)
- **Jitter**: 0.036 ms (exceptional timing consistency)
- **Reliability**: 98.5% delivery success (1.5% packet loss)
- **Throughput**: 1.95 Mbps with 98% efficiency
- **Performance**: Superior for real-time haptic control

#### DCCP Protocol Results  
- **Average Latency**: 2.45 ms (min: 1.98 ms, max: 3.84 ms)
- **Jitter**: 0.095 ms (moderate timing variability)
- **Reliability**: 86.6% delivery success (13.4% packet loss)
- **Throughput**: 1.68 Mbps with frequent rate adjustments
- **Performance**: Reliability limitations make it unsuitable for critical control

#### SCTP Protocol Results
- **Average Latency**: 5.231 ms (min: 4.412 ms, max: 6.978 ms)
- **Jitter**: 0.683 ms (significant timing variations)
- **Reliability**: 97.6% delivery success (2.4% packet loss)
- **Throughput**: 1.93 Mbps with excellent stability
- **Performance**: High reliability but latency unsuitable for haptic control

### Statistical Analysis
- **ANOVA Testing**: Confirmed statistically significant differences between protocols (p < 0.001)
- **Correlation Analysis**: Higher jitter correlated with reduced reliability (r = -0.6)
- **Trend Analysis**: QUIC maintained consistent performance; DCCP showed degradation over time

## System Architecture

```
[Human Operator] → [Geomagic Touch] → [ROS2 Driver] → [Protocol Client] → [Network] → [Protocol Server] → [Robot Control]
      ↓                   ↓               ↓               ↓               ↓              ↓              ↓
   Manual Input      6-DOF Position   omni_msgs/     Haptic Protocol   Controlled    omni_msgs/    Physical Robot
                     1000Hz Updates   OmniState      Implementation    Environment   OmniState     or Simulation
```

### Data Flow
1. **Haptic Input**: Geomagic Touch captures operator movements at 1000Hz
2. **ROS2 Conversion**: Device state converted to standardized `omni_msgs/OmniState` messages
3. **Protocol Transmission**: Messages serialized and transmitted via QUIC/DCCP/SCTP
4. **Server Processing**: Received data processed and republished to robot control topics
5. **Robot Control**: Commands forwarded to robotic system for execution

## Protocol Selection Framework

### Decision Criteria

**Choose QUIC when:**
- Sub-millisecond timing requirements
- High reliability needed (>98%)
- Haptic feedback applications
- Surgical or precision control

**Choose SCTP when:**
- Reliability more important than latency
- Industrial automation (slower control loops)
- Multi-path network redundancy needed
- Long-duration stable connections required

**Avoid DCCP for:**
- Critical control applications
- Systems requiring >90% reliability
- Real-time haptic feedback
- Safety-critical operations

## Academic Usage

### Citation
```bibtex
@mastersthesis{clerix2025protocols,
    title={Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control},
    author={Clerix, Warre},
    year={2025},
    school={UHasselt & KULeuven},
    type={MSc Thesis},
    note={Implementation and results available at: https://github.com/ClerixWarre/haptic-teleoperation-network-protocols}
}
```

### Research Contributions
- First comprehensive comparison of QUIC, DCCP, and SCTP for haptic teleoperation
- Quantitative performance benchmarks for real-time robotic control protocols
- Open-source implementation framework for protocol evaluation
- Evidence-based protocol selection guidelines for robotic applications

## Methodology

### Experimental Design
- **Hardware**: Intel Core Ultra 9 185H client, Intel Core i7-12700K server
- **Network**: Controlled Gigabit Ethernet (< 0.5ms latency, 0% loss)
- **Protocol Implementation**: Equivalent architectures with consistent message formats
- **Performance Monitoring**: Real-time latency, jitter, throughput, and reliability tracking
- **Statistical Analysis**: ANOVA, correlation analysis, and trend evaluation

### Key Features Tested
- **Message Serialization**: Consistent 75-byte haptic state messages
- **Priority Handling**: Emergency, high-priority, and normal message classifications
- **Multi-streaming**: Stream-based multiplexing for applicable protocols
- **Reliability Mechanisms**: Built-in vs. application-layer error handling

## Study Limitations

- **Network Conditions**: Focused on near-optimal conditions suitable for haptic control
- **Theoretical Analysis**: Robot control impact predictions based on network measurements
- **Single Hardware Setup**: One haptic device and computer configuration tested
- **Protocol Configurations**: Standard configurations used; advanced features not extensively tested

## Future Work

- **Extended Network Conditions**: Testing under challenging network environments
- **End-to-End Validation**: Complete robotic system performance measurements
- **Advanced Protocol Features**: QUIC connection migration, SCTP multi-homing evaluation
- **Multi-Client Scenarios**: Scalability testing with multiple concurrent users
- **Real-World Deployment**: Testing in actual surgical or industrial environments

## Acknowledgments

- **Thesis Supervisor***: Prof. dr. Nikolaos Tsiogkas, Hasselt University
- **Geomagic Touch ROS2 Integration**: Based on [IvoD1998/Geomagic_Touch_ROS2](https://github.com/IvoD1998/Geomagic_Touch_ROS2)
- **Research Supervision**: ACRO Research Group, UHasselt
- **Robot Control System**: Built upon previous ACRO research implementations
- **Technical Support**: UHasselt & KULeuven Faculty of Electromechanical Engineering Technology at Campus Diepenbeek

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

**Warre Clerix**  
MSc Student, UHasselt & KULeuven 
ACRO Research Group  
Faculty of Engineering Technology

---
*Part of MSc thesis research at UHasselt & KULeuven - ACRO Research Group*  
*Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control*
