# Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository contains the complete implementation and experimental results for the MSc thesis: **"Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control"** by Warre Clerix at UHasselt & KULeuven.

## Overview

This research evaluates three emerging network protocols (QUIC, DCCP, SCTP) for haptic teleoperation systems using a ROS2-based experimental platform with a Geomagic Touch haptic interface. The study focuses on real-time communication requirements for surgical robotics and precision control applications.

## Key Results

| Protocol | Avg Latency | Jitter | Reliability | Best Use Case |
|----------|-------------|---------|-------------|---------------|
| QUIC | 1.198 ms | 0.036 ms | 98.5% | Haptic Control |
| DCCP | 2.45 ms | 0.095 ms | 86.6% | Non-critical telemetry |
| SCTP | 5.231 ms | 0.683 ms | 97.6% | Industrial automation |

## Repository Structure

```
├── workspaces/          # ROS2 workspaces for each protocol
├── experimental_results/# Test results and analysis
├── scripts/            # Build and experiment scripts
├── thesis/             # Thesis documents
├── docs/               # Additional documentation
└── docker/             # Docker configuration
```

## Quick Start

1. **Install Dependencies**
   ```bash
   ./scripts/setup/install_dependencies.sh
   ```

2. **Build All Workspaces**
   ```bash
   ./scripts/build/build_all_workspaces.sh
   ```

3. **Run Experiments**
   See [docs/usage_guide.md](docs/usage_guide.md) for detailed instructions.

## Documentation

- [Installation Guide](docs/installation_guide.md)
- [Usage Guide](docs/usage_guide.md)
- [Protocol Comparison](docs/protocol_comparison.md)
- [Troubleshooting](docs/troubleshooting.md)

## Citation

```bibtex
@mastersthesis{clerix2025protocols,
  title={Comparative Analysis of QUIC and Other Network Protocols for Real-Time Robotic Control},
  author={Clerix, Warre},
  year={2025},
  school={UHasselt & KULeuven}
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
