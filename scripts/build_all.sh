#!/bin/bash

# Build script for haptic teleoperation protocol implementations
# MSc Thesis - Warre Cleric

echo "=== Building Haptic Teleoperation Protocol Implementations ==="

# Function to build protocol
build_protocol() {
    local protocol=$1
    echo "Building $protocol..."
    
    # Build client
    echo "  Building ${protocol}_haptic_client..."
    cd src/$protocol/client
    g++ -o ${protocol}_haptic_client ${protocol}_haptic_client.cpp \
        $(get_libs $protocol) -pthread -std=c++17
    
    # Build server  
    echo "  Building ${protocol}_haptic_server..."
    cd ../server
    g++ -o ${protocol}_haptic_server ${protocol}_haptic_server.cpp \
        $(get_libs $protocol) -pthread -std=c++17
    
    cd ../../..
    echo "  $protocol build complete!"
}

# Function to get protocol-specific libraries
get_libs() {
    case $1 in
        "quic") echo "-lmsquic" ;;
        "dccp") echo "" ;;
        "sctp") echo "-lsctp" ;;
    esac
}

# Build all protocols
build_protocol "quic"
build_protocol "dccp" 
build_protocol "sctp"

echo ""
echo "=== All builds complete! ==="
echo ""
echo "To run experiments:"
echo "1. Terminal 1: ros2 run geomagic_touch geomagic_node"
echo "2. Terminal 2: cd src/[protocol]/server && ./[protocol]_haptic_server"
echo "3. Terminal 3: cd src/[protocol]/client && ./[protocol]_haptic_client"
