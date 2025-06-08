# Network Protocol Evaluation for Real-Time Robotic Control

This repository contains the implementation and experimental results for the MSc thesis: **"Comparative Analysis of QUIC, DCCP, and SCTP Protocols for Real-Time Robotic Control Applications"** by Warre Cleric.

## Overview

Evaluation of QUIC, DCCP, and SCTP protocols for haptic teleoperation systems using ROS2 and Geomagic Touch interface.

## Key Results

| Protocol | Avg Latency | Jitter | Reliability | Best For |
|----------|-------------|--------|-------------|----------|
| **QUIC** | 1.198 ms | 0.036 ms | 98.5% | **Haptic Control** |
| **DCCP** | 2.45 ms | 0.095 ms | 86.6% | Non-critical telemetry |
| **SCTP** | 5.231 ms | 0.683 ms | 97.6% | Industrial automation |

## Repository Structure
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
│   ├── quic/ (client-logs.txt, server-logs.txt)
│   ├── dccp/ (client-logs.txt, server-logs.txt)
│   └── sctp/ (client-logs.txt, server-logs.txt)
└── scripts/build_all.sh

## Dependencies

- **ROS2 Humble** - Robot Operating System
- **Geomagic Touch ROS2 Driver**: [IvoD1998/Geomagic_Touch_ROS2](https://github.com/IvoD1998/Geomagic_Touch_ROS2)
- **MsQuic** - For QUIC protocol implementation
- **SCTP libraries** - For SCTP protocol implementation
- **Linux DCCP support** - For DCCP protocol implementation

## Quick Build & Run

### Build All Protocols
```bash
git clone https://github.com/ClerixWarre/haptic-teleoperation-network-protocols.git
cd haptic-teleoperation-network-protocols
chmod +x scripts/build_all.sh
./scripts/build_all.sh
