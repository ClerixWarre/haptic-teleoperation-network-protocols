#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
RESULTS_DIR="$ROOT_DIR/experimental_results"

echo "==============================================="
echo "Running all protocol experiments"
echo "==============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Default experiment duration (seconds)
DURATION=${1:-300}  # 5 minutes default, can be overridden by command line

# Function to check if a process is running
is_running() {
    kill -0 $1 2>/dev/null
}

# Function to run experiment for a protocol
run_protocol_experiment() {
    local protocol=$1
    local package_name="${protocol}_haptic_teleoperation"
    
    # Special case for DCCP which uses different package name
    if [ "$protocol" = "dccp" ]; then
        package_name="dccp_haptic"
    elif [ "$protocol" = "quic" ]; then
        package_name="quic_haptic"
    fi
    
    echo -e "\n${YELLOW}=== Starting $protocol experiment ===${NC}"
    echo "Duration: $DURATION seconds"
    echo "Package: $package_name"
    
    # Create log directory with timestamp
    local timestamp=$(date +%Y%m%d_%H%M%S)
    local log_dir="$RESULTS_DIR/logs/$protocol/$timestamp"
    mkdir -p "$log_dir"
    
    # Source ROS2 and workspace
    source /opt/ros/humble/setup.bash
    source "$ROOT_DIR/workspaces/${protocol}_haptic_ws/install/setup.bash"
    
    # Start server in background
    echo "Starting $protocol server..."
    ros2 run $package_name ${protocol}_haptic_server > \
        "$log_dir/${protocol}_server.log" 2>&1 &
    local server_pid=$!
    
    # Wait for server to start
    sleep 3
    
    if ! is_running $server_pid; then
        echo -e "${RED}Error: $protocol server failed to start!${NC}"
        echo "Check log at: $log_dir/${protocol}_server.log"
        return 1
    fi
    
    # Start client in background
    echo "Starting $protocol client..."
    ros2 run $package_name ${protocol}_haptic_client > \
        "$log_dir/${protocol}_client.log" 2>&1 &
    local client_pid=$!
    
    # Wait for client to start
    sleep 2
    
    if ! is_running $client_pid; then
        echo -e "${RED}Error: $protocol client failed to start!${NC}"
        echo "Check log at: $log_dir/${protocol}_client.log"
        kill $server_pid 2>/dev/null || true
        return 1
    fi
    
    # Monitor experiment
    echo "Running experiment..."
    local elapsed=0
    while [ $elapsed -lt $DURATION ]; do
        if ! is_running $server_pid || ! is_running $client_pid; then
            echo -e "${RED}Warning: Process died during experiment!${NC}"
            break
        fi
        
        # Progress indicator
        local progress=$((elapsed * 100 / DURATION))
        echo -ne "\rProgress: ["
        printf "%-50s" $(printf "#%.0s" $(seq 1 $((progress/2))))
        echo -ne "] $progress% ($elapsed/$DURATION s)"
        
        sleep 5
        elapsed=$((elapsed + 5))
    done
    echo ""  # New line after progress
    
    # Stop processes gracefully
    echo "Stopping $protocol experiment..."
    kill -SIGTERM $client_pid 2>/dev/null || true
    sleep 1
    kill -SIGTERM $server_pid 2>/dev/null || true
    
    # Wait for processes to finish
    wait $client_pid 2>/dev/null || true
    wait $server_pid 2>/dev/null || true
    
    # Generate summary
    echo "Generating summary..."
    {
        echo "Experiment Summary"
        echo "=================="
        echo "Protocol: $protocol"
        echo "Start time: $timestamp"
        echo "Duration: $DURATION seconds"
        echo "Server log: ${protocol}_server.log"
        echo "Client log: ${protocol}_client.log"
    } > "$log_dir/summary.txt"
    
    echo -e "${GREEN}✓ $protocol experiment complete!${NC}"
    echo "Results saved in: $log_dir"
    
    return 0
}

# Check dependencies
echo "Checking dependencies..."

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ROS2 not found! Please install ROS2 Humble.${NC}"
    exit 1
fi

# Check if Geomagic Touch driver is running
echo "Checking Geomagic Touch driver..."
source /opt/ros/humble/setup.bash
if ! timeout 5s ros2 node list 2>/dev/null | grep -q geomagic; then
    echo -e "${YELLOW}Warning: Geomagic Touch driver not detected!${NC}"
    echo "Please start it with: ros2 run geomagic_touch geomagic_node"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if workspaces are built
for ws in quic_haptic_ws dccp_haptic_ws sctp_haptic_ws; do
    if [ ! -d "$ROOT_DIR/workspaces/$ws/install" ]; then
        echo -e "${RED}Error: $ws not built!${NC}"
        echo "Please run: ./scripts/build/build_all_workspaces.sh"
        exit 1
    fi
done

# Run experiments
echo -e "\n${YELLOW}Starting experiments...${NC}"
echo "Each protocol will run for $DURATION seconds"
echo "Results will be saved to: $RESULTS_DIR/logs/"

TOTAL_START=$(date +%s)
SUCCESS_COUNT=0
FAIL_COUNT=0

# QUIC experiment
if run_protocol_experiment "quic"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# Wait between experiments
if [ $SUCCESS_COUNT -gt 0 ] && [ $((SUCCESS_COUNT + FAIL_COUNT)) -lt 3 ]; then
    echo -e "\n${YELLOW}Waiting 30 seconds before next experiment...${NC}"
    sleep 30
fi

# DCCP experiment
if run_protocol_experiment "dccp"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

# Wait between experiments
if [ $SUCCESS_COUNT -gt 0 ] && [ $((SUCCESS_COUNT + FAIL_COUNT)) -lt 3 ]; then
    echo -e "\n${YELLOW}Waiting 30 seconds before next experiment...${NC}"
    sleep 30
fi

# SCTP experiment
if run_protocol_experiment "sctp"; then
    ((SUCCESS_COUNT++))
else
    ((FAIL_COUNT++))
fi

TOTAL_END=$(date +%s)
TOTAL_DURATION=$((TOTAL_END - TOTAL_START))

# Final summary
echo -e "\n==============================================="
echo "All experiments complete!"
echo -e "${GREEN}Successful experiments: $SUCCESS_COUNT${NC}"
if [ $FAIL_COUNT -gt 0 ]; then
    echo -e "${RED}Failed experiments: $FAIL_COUNT${NC}"
fi
echo "Total time: $((TOTAL_DURATION / 60)) minutes $((TOTAL_DURATION % 60)) seconds"
echo "Results saved in: $RESULTS_DIR/logs/"
echo "==============================================="

# Create overall summary
{
    echo "Overall Experiment Summary"
    echo "========================="
    echo "Date: $(date)"
    echo "Total duration: $TOTAL_DURATION seconds"
    echo "Successful experiments: $SUCCESS_COUNT"
    echo "Failed experiments: $FAIL_COUNT"
    echo ""
    echo "Individual experiment duration: $DURATION seconds"
    echo ""
    echo "Experiments run:"
    for protocol in quic dccp sctp; do
        echo "- $protocol"
    done
} > "$RESULTS_DIR/logs/experiment_summary_$(date +%Y%m%d_%H%M%S).txt"

# Return error if any experiments failed
exit $FAIL_COUNT
