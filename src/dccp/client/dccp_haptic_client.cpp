#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <fstream>
#include <atomic>
#include <memory>
#include <csignal>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>
#include <linux/dccp.h>
#include <sys/types.h>

// ROS2 include files - always included now
#include <rclcpp/rclcpp.hpp>
#include <omni_msgs/msg/omni_state.hpp>

// Haptic message priority for surgical robotics
enum HapticMessagePriority {
    NORMAL = 0,
    HIGH = 1,
    EMERGENCY = 2  // For emergency stop or critical commands
};

// Logging levels
enum LogLevel {
    LOG_ERROR = 0,  // Only errors
    LOG_WARNING = 1, // Errors and warnings
    LOG_INFO = 2,    // General info, statistics
    LOG_DEBUG = 3    // Detailed debugging
};

// Global logging configuration
struct LogConfig {
    LogLevel level = LOG_INFO;  // Default to INFO level
    bool colorOutput = true;    // Enable colored output
    bool showTimestamps = true; // Show timestamps
    
    // How often to show periodic statistics (in messages)
    unsigned int statsInterval = 1000;
    
    // Enable specific logging categories
    bool enableRttLogging = true;
    bool enableDataLogging = false;
    bool enableStreamLogging = false;
} g_logConfig;

// Helper function for colored console output
void logMessage(LogLevel level, const std::string& message) {
    static std::mutex logMutex;
    static auto startTime = std::chrono::steady_clock::now();
    
    if (level > g_logConfig.level) {
        return; // Skip messages above current verbosity level
    }
    
    std::lock_guard<std::mutex> lock(logMutex);
    
    std::string prefix;
    std::string colorCode;
    std::string resetCode = g_logConfig.colorOutput ? "\033[0m" : "";
    
    // Generate timestamp if enabled
    std::string timestamp;
    if (g_logConfig.showTimestamps) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - startTime).count();
        std::stringstream ss;
        ss << "[" << std::fixed << std::setprecision(3) << elapsed << "s] ";
        timestamp = ss.str();
    }
    
    // Set color and prefix based on level
    switch (level) {
    case LOG_ERROR:
        prefix = "[ERROR] ";
        colorCode = g_logConfig.colorOutput ? "\033[1;31m" : ""; // Bold Red
        break;
    case LOG_WARNING:
        prefix = "[WARN]  ";
        colorCode = g_logConfig.colorOutput ? "\033[1;33m" : ""; // Bold Yellow
        break;
    case LOG_INFO:
        prefix = "[INFO]  ";
        colorCode = g_logConfig.colorOutput ? "\033[1;36m" : ""; // Bold Cyan
        break;
    case LOG_DEBUG:
        prefix = "[DEBUG] ";
        colorCode = g_logConfig.colorOutput ? "\033[1;35m" : ""; // Bold Magenta
        break;
    }
    
    std::cout << colorCode << timestamp << prefix << message << resetCode << std::endl;
}

// Convenience logging functions
void logError(const std::string& message) { logMessage(LOG_ERROR, message); }
void logWarning(const std::string& message) { logMessage(LOG_WARNING, message); }
void logInfo(const std::string& message) { logMessage(LOG_INFO, message); }
void logDebug(const std::string& message) { logMessage(LOG_DEBUG, message); }

// Global synchronization primitives
std::mutex mtx;
std::mutex messagesMutex;
std::condition_variable cv;
std::atomic<bool> connectionClosed{false};
std::atomic<bool> shutdownRequested{false};
std::atomic<bool> cleanupComplete{false};

// Client Configuration
const uint16_t ServerPort = 4433; // Same as QUIC implementation
const char* DEFAULT_SERVER_NAME = "localhost";

// DCCP specific configuration
const int DCCP_SERVICE_CODE = 42; // Custom service code for our application
const int MAX_DCCP_PACKET_SIZE = 1400; // Maximum datagram size we'll use

// Number of virtual streams to simulate (like QUIC implementation)
const int NUM_STREAMS = 4;
std::mutex g_streamMutexes[NUM_STREAMS];
std::atomic<bool> g_streamReady[NUM_STREAMS] = {false};
std::vector<uint8_t> g_pending_data[NUM_STREAMS]; // Separate pending data for each stream
std::mutex g_pending_mutex[NUM_STREAMS]; // Separate mutex for each stream's pending data

// DCCP socket for main connection
int g_dccpSocket = -1;
struct sockaddr_in g_serverAddr;

// Queue management settings for surgical applications
const size_t MAX_QUEUE_SIZE = 2000; // Increased for surgical data

// Global state for message sending
struct MessageSendState {
    std::atomic<bool> readyForNextMessage{true};
    std::atomic<size_t> currentIndex{0};
    std::atomic<bool> shutdownInitiated{false};
    std::mutex mutex;
    std::condition_variable cv;
};

// Structure to manage send buffer lifetime - optimized for surgical robotics
struct SendContext {
    std::vector<uint8_t> buffer;  // Directly own buffer (not via shared_ptr)
    uint64_t sequenceNumber;
    HapticMessagePriority priority;
    std::chrono::steady_clock::time_point sendTime;
    int streamIndex; // Which virtual stream this belongs to
    
    SendContext(const std::vector<uint8_t>& data, uint64_t seqNum, int stream, HapticMessagePriority prio = NORMAL)
        : buffer(data), sequenceNumber(seqNum), streamIndex(stream), priority(prio) {
        sendTime = std::chrono::steady_clock::now();
    }
};

// Global structures for tracking message timing
struct MessageTiming {
    std::chrono::steady_clock::time_point sendTime;
    bool responseReceived = false;
    double rttMs = 0;
    int streamIndex = -1; // Track which stream was used
};

// Global tracking map
std::mutex g_timing_mutex;
std::unordered_map<uint64_t, MessageTiming> g_message_timings;

// Global instance
MessageSendState g_sendState;

// Adaptive rate control parameters for surgical applications
std::atomic<int> g_currentMessageRate{1000}; // Start with 1000 Hz
const int MIN_MESSAGE_RATE = 200;  // Minimum acceptable rate for surgical control
const int MAX_MESSAGE_RATE = 1000; // Maximum rate
const double TARGET_SUCCESS_RATE = 0.95; // Target 95% success for surgical applications
std::chrono::microseconds g_messageInterval(1000000 / g_currentMessageRate);
std::chrono::steady_clock::time_point g_lastRateAdjustment = std::chrono::steady_clock::now();
const auto RATE_ADJUSTMENT_INTERVAL = std::chrono::seconds(2); // Adjust rate every 2 seconds

// Network performance metrics with improved calculation methods
struct NetworkMetrics {
    // Latency metrics
    std::vector<double> latencies;
    double minLatency = std::numeric_limits<double>::max();
    double maxLatency = 0;
    double avgLatency = 0;
    double latencySum = 0;  // Running sum for more efficient average calculation
    std::mutex latency_mutex; // Protect latency calculations
    
    // Throughput metrics
    std::atomic<uint64_t> totalBytesSent{0};
    std::atomic<uint64_t> totalBytesReceived{0};
    std::chrono::steady_clock::time_point startTime;
    
    // Packet metrics
    std::atomic<uint64_t> messagesSent{0};
    std::atomic<uint64_t> messagesReceived{0};
    std::atomic<uint64_t> messagesLost{0};
    std::atomic<uint64_t> messagesDuplicated{0};
    std::atomic<uint64_t> messagesDropped{0};  // Queue overflow
    
    // Stream-specific metrics
    std::atomic<uint64_t> messagesSentPerStream[NUM_STREAMS]{0};
    std::atomic<uint64_t> messagesReceivedPerStream[NUM_STREAMS]{0};
    
    // Real-time measurement
    std::atomic<uint64_t> currentThroughputBps{0};
    std::atomic<double> currentLatencyMs{0};
    std::atomic<double> avgJitterMs{0};
    double cumulativeJitter = 0;
    double lastLatencyMs = 0;
    
    NetworkMetrics() {
        startTime = std::chrono::steady_clock::now();
    }
    
    void addLatencySample(double latencyMs, int streamIndex = -1) {
        std::lock_guard<std::mutex> lock(latency_mutex);
        
        // Limit vector size to prevent unbounded growth
        if (latencies.size() >= 1000) {
            latencySum -= latencies[0];
            latencies.erase(latencies.begin());
        }
        
        latencies.push_back(latencyMs);
        latencySum += latencyMs;
        minLatency = std::min(minLatency, latencyMs);
        maxLatency = std::max(maxLatency, latencyMs);
        
        // More efficient average calculation
        avgLatency = latencySum / latencies.size();
        
        // Calculate jitter (variation in latency)
        if (latencies.size() > 1) {
            double jitter = std::abs(latencyMs - lastLatencyMs);
            cumulativeJitter += jitter;
            avgJitterMs.store(cumulativeJitter / (latencies.size() - 1));
        }
        
        lastLatencyMs = latencyMs;
        currentLatencyMs.store(latencyMs);
    }    
    
    void updateThroughput() {
        auto now = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(now - startTime).count();
        
        if (seconds > 0) {
            // Total throughput in bits per second
            double totalThroughputBps = (totalBytesSent * 8) / seconds;
            currentThroughputBps = static_cast<uint64_t>(totalThroughputBps);
        }
    }
    
    void printMetrics() {
        updateThroughput();
        std::stringstream ss;
        ss << "\n===== NETWORK METRICS =====";
        ss << "\nMessages sent: " << messagesSent.load();
        ss << "\nMessages received: " << messagesReceived.load();
        
        // Calculate packet loss rate
        uint64_t sentMsgs = messagesSent.load();
        uint64_t receivedMsgs = messagesReceived.load();
        
        if (sentMsgs > 0) {
            double lossRate = ((sentMsgs - receivedMsgs) * 100.0) / sentMsgs;
            ss << "\nMessages lost: " << (sentMsgs - receivedMsgs) << " (" << std::fixed << std::setprecision(2) << lossRate << "%)";
        } else {
            ss << "\nMessages lost: 0 (0%)";
        }
        
        if (messagesDuplicated > 0) {
            ss << "\nDuplicated messages: " << messagesDuplicated.load();
        }
        
        if (messagesDropped > 0) {
            ss << "\nDropped due to queue overflow: " << messagesDropped.load();
        }
        
        ss << "\nTotal data sent: " << totalBytesSent.load() << " bytes";
        ss << "\nTotal data received: " << totalBytesReceived.load() << " bytes";
        ss << "\nThroughput: " << (currentThroughputBps / 1000.0) << " Kbps";
        
        // Per-stream statistics
        ss << "\n\nPer-Stream Statistics:";
        for (int i = 0; i < NUM_STREAMS; i++) {
            ss << "\n  Stream " << i << ": Sent " << messagesSentPerStream[i].load() 
               << ", Received " << messagesReceivedPerStream[i].load();
            
            double streamSuccessRate = 0;
            if (messagesSentPerStream[i] > 0) {
                streamSuccessRate = (messagesReceivedPerStream[i] * 100.0) / messagesSentPerStream[i];
                ss << " (Success: " << std::fixed << std::setprecision(1) << streamSuccessRate << "%)";
            }
        }
        
        std::lock_guard<std::mutex> lock(latency_mutex);
        if (!latencies.empty()) {
            ss << "\nLatency (min/avg/max): " << std::fixed << std::setprecision(3) 
               << minLatency << " / " << avgLatency << " / " << maxLatency << " ms";
            ss << "\nAverage jitter: " << std::fixed << std::setprecision(5) << avgJitterMs << " ms";
        }
        
        ss << "\nCurrent message rate: " << g_currentMessageRate << " Hz";
        ss << "\n===========================";
        
        logInfo(ss.str());
    }
    
    // Method for compact progress updates during operation
    void printCompactStatus() {
        // Only print if we have some messages
        if (messagesSent == 0) return;
        
        std::stringstream ss;
        
        // Calculate time running
        auto now = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(now - startTime).count();
        
        // Message rate
        double msgRate = messagesSent / seconds;
        
        // Calculate reception rate if applicable
        double receiveRate = 0;
        if (messagesSent > 0) {
            receiveRate = (messagesReceived * 100.0) / messagesSent;
        }
        
        // Current latency if available
        std::string latencyStr = "N/A";
        if (!latencies.empty()) {
            std::stringstream latSs;
            latSs << std::fixed << std::setprecision(2) << currentLatencyMs.load();
            latencyStr = latSs.str();
        }
        
        // Format the status line
        ss << "[Status] Msgs: " << messagesSent.load() << " @ " 
           << std::fixed << std::setprecision(0) << msgRate << "/s | "
           << "Success: " << std::fixed << std::setprecision(1) << receiveRate << "% | "
           << "Latency: " << latencyStr << " ms";
        
        logInfo(ss.str());
    }
};

// Function to print only essential metrics
void printEssentialMetrics(const NetworkMetrics& metrics) {
    std::stringstream ss;
    ss << "STATS | ";
    
    // Calculate time
    auto now = std::chrono::steady_clock::now();
    double totalTimeSeconds = std::chrono::duration<double>(now - metrics.startTime).count();
    
    // Message throughput
    double messageRate = metrics.messagesSent / totalTimeSeconds;
    ss << "Msg: " << metrics.messagesSent.load() << " (" << std::fixed << std::setprecision(1) 
       << messageRate << "/s) | ";
    
    // Success rate
    if (metrics.messagesSent > 0) {
        double successRate = (metrics.messagesReceived * 100.0) / metrics.messagesSent;
        ss << "Success: " << std::fixed << std::setprecision(2) << successRate << "% | ";
    }
    
    // Data throughput
    double dataRateMbps = (metrics.totalBytesSent * 8) / (totalTimeSeconds * 1000000);
    ss << "Rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps | ";
    
    // Latency stats if available
    if (!metrics.latencies.empty()) {
        ss << "RTT: " << std::fixed << std::setprecision(2) 
           << metrics.minLatency << "/" << metrics.avgLatency << "/" 
           << metrics.maxLatency << " ms | ";
        ss << "Jitter: " << std::fixed << std::setprecision(3) << metrics.avgJitterMs << " ms";
    }
    
    logInfo(ss.str());
}

// Global network metrics
NetworkMetrics g_metrics;

// Message structure with timestamp and priority
struct MessageWithTimestamp {
    std::vector<uint8_t> data;
    std::chrono::steady_clock::time_point sendTime;
    uint64_t sequenceNumber;
    HapticMessagePriority priority = NORMAL;
};

// Global queue for real-time data
std::mutex g_data_mutex;
std::queue<MessageWithTimestamp> g_message_queue;
std::condition_variable g_queue_cv;
std::atomic<uint64_t> g_sequence_number{0};

// Forward declarations
void dumpHexData(const uint8_t* data, uint32_t length);
void StartRealTimeDataThread();
void updateMessageRate();

// Global ROS2 variables
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr g_subscription = nullptr;
std::atomic<bool> g_ros_initialized{false};
std::mutex g_ros_mutex;

// Context structure for a stream
struct StreamContext {
    int streamIndex;
    StreamContext(int index) : streamIndex(index) {}
};

// DCCP specific functions

// Create and setup DCCP socket
bool setupDCCPSocket(const std::string& serverName, uint16_t port) {
    logInfo("Setting up DCCP socket to " + serverName + ":" + std::to_string(port));

    // Create DCCP socket
    g_dccpSocket = socket(AF_INET, SOCK_DCCP, IPPROTO_DCCP);
    if (g_dccpSocket < 0) {
        logError("Failed to create DCCP socket: " + std::string(strerror(errno)));
        return false;
    }

    // Set DCCP service code
    int service_code = htonl(DCCP_SERVICE_CODE);
    if (setsockopt(g_dccpSocket, SOL_DCCP, DCCP_SOCKOPT_SERVICE, &service_code, sizeof(service_code)) < 0) {
        logError("Failed to set DCCP service code: " + std::string(strerror(errno)));
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }

    // Set non-blocking mode
    int flags = fcntl(g_dccpSocket, F_GETFL, 0);
    if (flags < 0) {
        logError("Failed to get socket flags: " + std::string(strerror(errno)));
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }
    if (fcntl(g_dccpSocket, F_SETFL, flags | O_NONBLOCK) < 0) {
        logError("Failed to set socket to non-blocking: " + std::string(strerror(errno)));
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }

    // Set up server address
    struct hostent *he = gethostbyname(serverName.c_str());
    if (!he) {
        logError("Failed to resolve host: " + serverName);
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }

    memset(&g_serverAddr, 0, sizeof(g_serverAddr));
    g_serverAddr.sin_family = AF_INET;
    g_serverAddr.sin_port = htons(port);
    memcpy(&g_serverAddr.sin_addr, he->h_addr_list[0], he->h_length);

    // Connect to server
    logInfo("Connecting to DCCP server at " + serverName + ":" + std::to_string(port));
    if (connect(g_dccpSocket, (struct sockaddr*)&g_serverAddr, sizeof(g_serverAddr)) < 0) {
        if (errno != EINPROGRESS) {
            logError("Failed to connect to server: " + std::string(strerror(errno)));
            close(g_dccpSocket);
            g_dccpSocket = -1;
            return false;
        }

        // Wait for connection to complete
        struct pollfd pfd;
        pfd.fd = g_dccpSocket;
        pfd.events = POLLOUT;
        int ret = poll(&pfd, 1, 5000); // 5 second timeout
        
        if (ret <= 0) {
            logError("Connection to server timed out");
            close(g_dccpSocket);
            g_dccpSocket = -1;
            return false;
        }

        // Check if connection succeeded
        int err;
        socklen_t len = sizeof(err);
        if (getsockopt(g_dccpSocket, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
            logError("Connection failed: " + std::string(strerror(err)));
            close(g_dccpSocket);
            g_dccpSocket = -1;
            return false;
        }
    }

    // Connection successful
    logInfo("Connected to DCCP server successfully");

    // Set socket buffer sizes
    int sendbuf = 512 * 1024; // 512KB
    int recvbuf = 512 * 1024; // 512KB
    
    if (setsockopt(g_dccpSocket, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf)) < 0) {
        logWarning("Failed to set send buffer size: " + std::string(strerror(errno)));
    }
    
    if (setsockopt(g_dccpSocket, SOL_SOCKET, SO_RCVBUF, &recvbuf, sizeof(recvbuf)) < 0) {
        logWarning("Failed to set receive buffer size: " + std::string(strerror(errno)));
    }

    // Set CCID (Congestion Control ID) to CCID 2 (TCP-like)
    int ccid = 2; // CCID 2 is TCP-like congestion control
    if (setsockopt(g_dccpSocket, SOL_DCCP, DCCP_SOCKOPT_CCID, &ccid, sizeof(ccid)) < 0) {
        logWarning("Failed to set CCID: " + std::string(strerror(errno)));
    }

    // Mark all virtual streams as ready
    for (int i = 0; i < NUM_STREAMS; i++) {
        g_streamReady[i] = true;
    }

    return true;
}

// Callback for omni_state topic with clean logging
void omniStateCallback(const std::shared_ptr<const omni_msgs::msg::OmniState> msg) {
    try {
        // Create a buffer to store the serialized message
        MessageWithTimestamp message;
        message.sendTime = std::chrono::steady_clock::now();
        message.sequenceNumber = g_sequence_number++;
        
        // Determine message priority based on haptic data
        // For surgical applications, detect emergency conditions
        // Example: if forces exceed threshold, mark as emergency
        float forceThreshold = 20.0f; // Example threshold
        float force = std::sqrt(
            msg->current.x * msg->current.x + 
            msg->current.y * msg->current.y + 
            msg->current.z * msg->current.z);
            
        if (force > forceThreshold || msg->locked) {
            message.priority = EMERGENCY;
        }
        else if (force > forceThreshold * 0.7) {
            message.priority = HIGH;
        }
        
        // Simple serialization - create fixed size message
        const size_t bufferSize = 256; // Adjust based on your message size
        message.data.resize(bufferSize);
        
        // First 8 bytes: sequence number
        memcpy(message.data.data(), &message.sequenceNumber, sizeof(uint64_t));

        // Next 8 bytes: timestamp
        uint64_t absoluteTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            message.sendTime.time_since_epoch()).count();
        memcpy(message.data.data() + sizeof(uint64_t), &absoluteTimestamp, sizeof(uint64_t));
        
        // Store timing information for RTT calculation
        {
            std::lock_guard<std::mutex> lock(g_timing_mutex);
            g_message_timings[message.sequenceNumber] = {message.sendTime, false, 0, -1}; // Stream index set when sent
            
            // Cleanup old entries (older than 30 seconds)
            auto now = std::chrono::steady_clock::now();
            auto it = g_message_timings.begin();
            while (it != g_message_timings.end()) {
                if (now - it->second.sendTime > std::chrono::seconds(30)) {
                    it = g_message_timings.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
        // Add logging for debugging, but only occasionally
        if (g_logConfig.enableDataLogging && message.sequenceNumber % 1000 == 0) {
            logDebug("Preparing message #" + std::to_string(message.sequenceNumber) + 
                   ", timestamp: " + std::to_string(absoluteTimestamp) +
                   ", priority: " + std::to_string(message.priority));
        }
        
        // Add message data - copy position values
        float x = static_cast<float>(msg->pose.position.x);
        float y = static_cast<float>(msg->pose.position.y);
        float z = static_cast<float>(msg->pose.position.z);
        
        memcpy(message.data.data() + 16, &x, sizeof(float));
        memcpy(message.data.data() + 20, &y, sizeof(float));
        memcpy(message.data.data() + 24, &z, sizeof(float));
        
        // Add quaternion data
        float qw = static_cast<float>(msg->pose.orientation.w);
        float qx = static_cast<float>(msg->pose.orientation.x);
        float qy = static_cast<float>(msg->pose.orientation.y);
        float qz = static_cast<float>(msg->pose.orientation.z);
        
        memcpy(message.data.data() + 28, &qw, sizeof(float));
        memcpy(message.data.data() + 32, &qx, sizeof(float));
        memcpy(message.data.data() + 36, &qy, sizeof(float));
        memcpy(message.data.data() + 40, &qz, sizeof(float));
        
        // Add velocity data
        float vx = static_cast<float>(msg->velocity.x);
        float vy = static_cast<float>(msg->velocity.y);
        float vz = static_cast<float>(msg->velocity.z);
        
        memcpy(message.data.data() + 44, &vx, sizeof(float));
        memcpy(message.data.data() + 48, &vy, sizeof(float));
        memcpy(message.data.data() + 52, &vz, sizeof(float));
        
        // Add current data
        float cx = static_cast<float>(msg->current.x);
        float cy = static_cast<float>(msg->current.y);
        float cz = static_cast<float>(msg->current.z);
        
        memcpy(message.data.data() + 56, &cx, sizeof(float));
        memcpy(message.data.data() + 60, &cy, sizeof(float));
        memcpy(message.data.data() + 64, &cz, sizeof(float));
        
        // Add boolean states
        uint8_t lockState = msg->locked ? 1 : 0;
        uint8_t gripperState = msg->close_gripper ? 1 : 0;
        
        memcpy(message.data.data() + 68, &lockState, sizeof(uint8_t));
        memcpy(message.data.data() + 69, &gripperState, sizeof(uint8_t));
        
        // Add priority
        uint8_t priorityValue = static_cast<uint8_t>(message.priority);
        memcpy(message.data.data() + 70, &priorityValue, sizeof(uint8_t));
        
        // Add to queue for sending with queue size management
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            
            // If queue is too large, drop the oldest message
            if (g_message_queue.size() >= MAX_QUEUE_SIZE) {
                g_message_queue.pop();
                g_metrics.messagesDropped++;
                
                // Only log occasional drops to prevent console spam
                if (g_metrics.messagesDropped % 100 == 0) {
                    logWarning("Queue overflow, dropped oldest message. Total dropped: " + 
                             std::to_string(g_metrics.messagesDropped.load()));
                }
            }
            
            g_message_queue.push(std::move(message));
        }
        g_queue_cv.notify_one();
        
        // Log position data occasionally for verification
        if (g_logConfig.enableDataLogging && g_sequence_number % 1000 == 0) {
            logDebug("Processed haptic data - position: " + 
                   std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + 
                   " velocity: " + std::to_string(vx) + ", " + std::to_string(vy) + ", " + std::to_string(vz));
        }
        
    } catch (const std::exception& e) {
        logError("Error in omni state callback: " + std::string(e.what()));
    }
}

// Initialize ROS2 for real-time data
bool InitializeROS() {
    std::lock_guard<std::mutex> lock(g_ros_mutex);
    
    if (g_ros_initialized) return true;
    
    try {
        // Initialize ROS2
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create node
        g_node = std::make_shared<rclcpp::Node>("dccp_haptic_client");
        
        // Create subscription to omni_state topic
        g_subscription = g_node->create_subscription<omni_msgs::msg::OmniState>(
            "/phantom/state", 10, omniStateCallback);
            
        // Start a thread for ROS spinning
        std::thread([&]() {
            logInfo("ROS2 spin thread started");
            while (!shutdownRequested && rclcpp::ok()) {
                try {
                    rclcpp::spin_some(g_node);
                } catch (const std::exception& e) {
                    logError("Error in ROS2 spin: " + std::string(e.what()));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            logInfo("ROS2 spin thread stopping");
        }).detach();
        
        g_ros_initialized = true;
        logInfo("ROS2 initialized for real-time data");
        return true;
    } catch (const std::exception& e) {
        logError("Error initializing ROS2: " + std::string(e.what()));
        return false;
    }
}

// Shutdown ROS2
void ShutdownROS() {
    std::lock_guard<std::mutex> lock(g_ros_mutex);
    
    if (g_ros_initialized) {
        try {
            // Reset subscription and node
            g_subscription.reset();
            g_node.reset();
            
            // Shutdown ROS
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            
            g_ros_initialized = false;
            logInfo("ROS2 shutdown complete");
        } catch (const std::exception& e) {
            logError("Error shutting down ROS2: " + std::string(e.what()));
        }
    }
}

// Function to dynamically adjust message rate based on success rate
void updateMessageRate() {
    auto now = std::chrono::steady_clock::now();
    
    // Only adjust rate periodically
    if (now - g_lastRateAdjustment < RATE_ADJUSTMENT_INTERVAL) {
        return;
    }
    
    g_lastRateAdjustment = now;
    
    // Calculate current success rate
    double successRate = 0;
    if (g_metrics.messagesSent > 0) {
        successRate = static_cast<double>(g_metrics.messagesReceived) / g_metrics.messagesSent;
    }
    
    // Previous rate for logging
    int oldRate = g_currentMessageRate;
    
    // Adjust rate based on success rate
    if (successRate < 0.80) {
        // Significantly reduce rate if success is poor
        g_currentMessageRate = std::max(MIN_MESSAGE_RATE, 
                                      g_currentMessageRate.load() - 100);
    }
    else if (successRate < TARGET_SUCCESS_RATE) {
        // Slightly reduce rate
        g_currentMessageRate = std::max(MIN_MESSAGE_RATE, 
                                      g_currentMessageRate.load() - 50);
    }
    else if (successRate > 0.98 && g_currentMessageRate < MAX_MESSAGE_RATE) {
        // Cautiously increase rate if doing very well
        g_currentMessageRate = std::min(MAX_MESSAGE_RATE, 
                                      g_currentMessageRate.load() + 50);
    }
    
    // Update message interval if rate changed
    if (oldRate != g_currentMessageRate) {
        g_messageInterval = std::chrono::microseconds(1000000 / g_currentMessageRate);
        logInfo("Rate adjusted: " + std::to_string(oldRate) + " -> " + 
               std::to_string(g_currentMessageRate) + " Hz (Success rate: " + 
               std::to_string(successRate * 100) + "%)");
    }
}

// Read thread for receiving messages from server
void StartReadThread() {
    std::thread([&]() {
        logInfo("Read thread started");
        
        uint8_t buffer[MAX_DCCP_PACKET_SIZE];
        
        while (!shutdownRequested.load() && g_dccpSocket >= 0) {
            // Poll for data
            struct pollfd pfd;
            pfd.fd = g_dccpSocket;
            pfd.events = POLLIN;
            int ret = poll(&pfd, 1, 100); // 100ms timeout
            
            if (ret < 0) {
                if (errno != EINTR) {
                    logError("Poll error: " + std::string(strerror(errno)));
                    break;
                }
                continue;
            }
            
            if (ret == 0) {
                // Timeout, check if shutdown requested
                continue;
            }
            
            // Receive data
            int streamIndex = -1;
            socklen_t addrLen = sizeof(g_serverAddr);
            ssize_t bytesRead = recvfrom(g_dccpSocket, buffer, sizeof(buffer), 0, 
                                          (struct sockaddr*)&g_serverAddr, &addrLen);
            
            if (bytesRead < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    logError("Error receiving data: " + std::string(strerror(errno)));
                    if (errno == ECONNRESET || errno == EPIPE) {
                        logError("Connection lost");
                        connectionClosed = true;
                        cv.notify_all();
                        break;
                    }
                }
                continue;
            }
            
            if (bytesRead == 0) {
                // Connection closed by server
                logInfo("Connection closed by server");
                connectionClosed = true;
                cv.notify_all();
                break;
            }
            
            // Check if we have enough data for at least header (message format: [streamIndex(4), seqNum(8), timestamp(8)])
            if (bytesRead < 20) {
                logWarning("Received packet too small: " + std::to_string(bytesRead) + " bytes");
                continue;
            }
            
            // Extract the stream index from the first 4 bytes
            memcpy(&streamIndex, buffer, sizeof(int));
            
            if (streamIndex < 0 || streamIndex >= NUM_STREAMS) {
                logWarning("Invalid stream index in received packet: " + std::to_string(streamIndex));
                continue;
            }
            
            // Extract sequence number and timestamp
            uint64_t sequenceNumber, timestamp;
            memcpy(&sequenceNumber, buffer + 4, sizeof(uint64_t));
            memcpy(&timestamp, buffer + 12, sizeof(uint64_t));
            
            if (g_logConfig.enableDataLogging && g_metrics.messagesReceived % 100 == 0) {
                logDebug("Stream " + std::to_string(streamIndex) + 
                         " received response: Seq=" + std::to_string(sequenceNumber) + 
                         ", Timestamp=" + std::to_string(timestamp));
            }
            
            // Process the acknowledgment
            auto now = std::chrono::steady_clock::now();
            
            bool matchFound = false;
            {
                std::lock_guard<std::mutex> lock(g_timing_mutex);
                auto it = g_message_timings.find(sequenceNumber);
                if (it != g_message_timings.end()) {
                    auto& timing = it->second;
                    timing.responseReceived = true;
                    timing.rttMs = std::chrono::duration<double, std::milli>(now - timing.sendTime).count();
                    
                    g_metrics.messagesReceived++;
                    g_metrics.messagesReceivedPerStream[streamIndex]++;
                    g_metrics.totalBytesReceived += bytesRead;
                    g_metrics.addLatencySample(timing.rttMs, streamIndex);
                    
                    if (g_logConfig.enableRttLogging &&
                        g_metrics.messagesReceived % g_logConfig.statsInterval == 0) {
                        logInfo("RTT for seq #" + std::to_string(sequenceNumber) +
                                ": " + std::to_string(timing.rttMs) + " ms");
                    }
                    
                    g_message_timings.erase(it);
                    matchFound = true;
                }
                
                if (!matchFound) {
                    g_metrics.messagesReceived++;
                    g_metrics.messagesReceivedPerStream[streamIndex]++;
                    g_metrics.totalBytesReceived += bytesRead;
                    double estimatedRtt = g_metrics.latencies.empty() ? 0.5 : g_metrics.avgLatency;
                    g_metrics.addLatencySample(estimatedRtt, streamIndex);
                    
                    if (g_metrics.messagesReceived % 5000 == 0) {
                        logDebug("Note: Some sequence numbers not matched (" +
                                 std::to_string(g_message_timings.size()) +
                                 " pending entries)");
                    }
                    
                    if (g_metrics.messagesReceived % 1000 == 0) {
                        auto oldestAllowed = now - std::chrono::seconds(5);
                        size_t removedCount = 0;
                        auto it = g_message_timings.begin();
                        while (it != g_message_timings.end()) {
                            if (it->second.sendTime < oldestAllowed) {
                                it = g_message_timings.erase(it);
                                removedCount++;
                            } else {
                                ++it;
                            }
                        }
                        if (removedCount > 0 && g_logConfig.enableDataLogging) {
                            logDebug("Cleaned up " + std::to_string(removedCount) +
                                     " stale entries from timing map");
                        }
                    }
                }
            }
            
            if (g_metrics.messagesReceived % g_logConfig.statsInterval == 0) {
                printEssentialMetrics(g_metrics);
            }
        }
        
        logInfo("Read thread stopping");
    }).detach();
}

// Start real-time data sending thread
void StartRealTimeDataThread() {
    logInfo("Starting real-time data thread with multi-stream approach");
    
    // Start the read thread
    StartReadThread();
    
    // Now process messages in real-time data thread
    std::thread([&]() {
        logDebug("Message processing thread started");
        
        // Flow control: limit messages per second - now adaptive
        auto lastSendTime = std::chrono::steady_clock::now();
        
        // Add periodic status reporting
        auto lastStatusTime = std::chrono::steady_clock::now();
        const auto STATUS_INTERVAL = std::chrono::seconds(5);
        
        while (!shutdownRequested.load() && !g_sendState.shutdownInitiated.load() && g_dccpSocket >= 0) {
            // Check if it's time for a status update
            auto currentTime = std::chrono::steady_clock::now();
            if (currentTime - lastStatusTime > STATUS_INTERVAL) {
                g_metrics.printCompactStatus();
                lastStatusTime = currentTime;
                
                // Periodic rate adjustment
                updateMessageRate();
            }
            
            // Wait for notification or timeout if queue is empty
            {
                std::unique_lock<std::mutex> lock(g_data_mutex);
                if (g_message_queue.empty()) {
                    // Only wait if the queue is empty
                    g_queue_cv.wait_for(lock, std::chrono::milliseconds(10), 
                        []{ return !g_message_queue.empty() || shutdownRequested.load(); });
                }
                
                // If shutdown requested or still empty after wait, skip to next iteration
                if (shutdownRequested.load() || g_sendState.shutdownInitiated.load() || g_message_queue.empty()) {
                    continue;
                }
            }
            
            // Find messages by priority using a temporary vector to work with
            std::vector<MessageWithTimestamp> messages;
            bool hasEmergencyMessage = false;
            bool hasHighPriorityMessage = false;
            MessageWithTimestamp emergencyMessage;
            MessageWithTimestamp highPriorityMessage;
            MessageWithTimestamp normalMessage;
            
            {
                std::unique_lock<std::mutex> lock(g_data_mutex);
                
                // Copy messages from queue to vector for processing
                size_t messagesToProcess = std::min(g_message_queue.size(), size_t(20)); // Process at most 20 at a time
                
                for (size_t i = 0; i < messagesToProcess; i++) {
                    messages.push_back(g_message_queue.front());
                    g_message_queue.pop();
                }
            }
            
            // Now search in the vector for priority messages
            auto emergencyIt = std::find_if(messages.begin(), messages.end(),
                [](const MessageWithTimestamp& msg) { return msg.priority == EMERGENCY; });
                
            if (emergencyIt != messages.end()) {
                emergencyMessage = *emergencyIt;
                messages.erase(emergencyIt);
                hasEmergencyMessage = true;
            } else {
                // Find high priority messages
                auto highPriorityIt = std::find_if(messages.begin(), messages.end(),
                    [](const MessageWithTimestamp& msg) { return msg.priority == HIGH; });
                
                if (highPriorityIt != messages.end()) {
                    highPriorityMessage = *highPriorityIt;
                    messages.erase(highPriorityIt);
                    hasHighPriorityMessage = true;
                } else if (!messages.empty()) {
                    // Get a normal message
                    normalMessage = messages.front();
                    messages.erase(messages.begin());
                }
            }
            
            // Put remaining messages back into the queue
            if (!messages.empty()) {
                std::unique_lock<std::mutex> lock(g_data_mutex);
                for (auto it = messages.rbegin(); it != messages.rend(); ++it) {
                    g_message_queue.push(*it);
                }
            }
            
            // Apply rate limiting only for normal messages
            auto timeSinceLastSend = currentTime - lastSendTime;
            
            // Process based on priority
            if (hasEmergencyMessage) {
                // Emergency messages bypass rate limiter
                lastSendTime = currentTime; // Update for next normal message
                
                // Select a stream for emergency message
                int streamIndex = 0; // Use first stream for emergency messages
                
                if (g_dccpSocket >= 0) {
                    // Prepend stream index to the message
                    std::vector<uint8_t> packet(4 + emergencyMessage.data.size());
                    memcpy(packet.data(), &streamIndex, sizeof(int));
                    memcpy(packet.data() + 4, emergencyMessage.data.data(), emergencyMessage.data.size());
                    
                    // Record which stream is used
                    {
                        std::lock_guard<std::mutex> lock(g_timing_mutex);
                        auto it = g_message_timings.find(emergencyMessage.sequenceNumber);
                        if (it != g_message_timings.end()) {
                            it->second.streamIndex = streamIndex;
                        }
                    }
                    
                    // Send data
                    ssize_t bytesSent = send(g_dccpSocket, packet.data(), packet.size(), 0);
                    
                    if (bytesSent < 0) {
                        logError("Failed to send emergency message: " + std::string(strerror(errno)));
                        
                        // Check if connection is lost
                        if (errno == ECONNRESET || errno == EPIPE) {
                            logError("Connection lost");
                            connectionClosed = true;
                            cv.notify_all();
                            break;
                        }
                    } else {
                        // Update metrics
                        g_metrics.messagesSent++;
                        g_metrics.messagesSentPerStream[streamIndex]++;
                        g_metrics.totalBytesSent += bytesSent;
                        
                        // Log emergency message sent
                        logInfo("Sent EMERGENCY message #" + std::to_string(emergencyMessage.sequenceNumber) + 
                               " on stream " + std::to_string(streamIndex));
                    }
                }
            }
            else if (hasHighPriorityMessage) {
                // High priority messages get mild rate limiting
                if (timeSinceLastSend < g_messageInterval / 2) {
                    std::this_thread::sleep_for(g_messageInterval / 2 - timeSinceLastSend);
                }
                lastSendTime = std::chrono::steady_clock::now(); // Update timestamp after sleeping
                
                // Use least loaded stream for high priority message
                int streamIndex = 0;
                uint64_t minMessages = g_metrics.messagesSentPerStream[0];
                
                for (int i = 1; i < NUM_STREAMS; i++) {
                    if (g_metrics.messagesSentPerStream[i] < minMessages) {
                        streamIndex = i;
                        minMessages = g_metrics.messagesSentPerStream[i];
                    }
                }
                
                if (g_dccpSocket >= 0) {
                    // Prepend stream index to the message
                    std::vector<uint8_t> packet(4 + highPriorityMessage.data.size());
                    memcpy(packet.data(), &streamIndex, sizeof(int));
                    memcpy(packet.data() + 4, highPriorityMessage.data.data(), highPriorityMessage.data.size());
                    
                    // Record which stream is used
                    {
                        std::lock_guard<std::mutex> lock(g_timing_mutex);
                        auto it = g_message_timings.find(highPriorityMessage.sequenceNumber);
                        if (it != g_message_timings.end()) {
                            it->second.streamIndex = streamIndex;
                        }
                    }
                    
                    // Send data
                    ssize_t bytesSent = send(g_dccpSocket, packet.data(), packet.size(), 0);
                    
                    if (bytesSent < 0) {
                        logError("Failed to send high priority message: " + std::string(strerror(errno)));
                        
                        // Check if connection is lost
                        if (errno == ECONNRESET || errno == EPIPE) {
                            logError("Connection lost");
                            connectionClosed = true;
                            cv.notify_all();
                            break;
                        }
                    } else {
                        // Update metrics
                        g_metrics.messagesSent++;
                        g_metrics.messagesSentPerStream[streamIndex]++;
                        g_metrics.totalBytesSent += bytesSent;
                        
                        // Log occasionally
                        if (g_metrics.messagesSent % g_logConfig.statsInterval == 0) {
                            logInfo("Sent HIGH PRIORITY message #" + 
                                   std::to_string(highPriorityMessage.sequenceNumber) + 
                                   " on stream " + std::to_string(streamIndex));
                        }
                    }
                }
            }
            else {
                // Normal message - apply full rate limiting
                if (timeSinceLastSend < g_messageInterval) {
                    std::this_thread::sleep_for(g_messageInterval - timeSinceLastSend);
                }
                lastSendTime = std::chrono::steady_clock::now(); // Update timestamp after sleeping
                
                // Round-robin stream selection for normal messages
                static int roundRobinIndex = 0;
                int streamIndex = (roundRobinIndex++ % NUM_STREAMS);
                
                if (g_dccpSocket >= 0) {
                    // Prepend stream index to the message
                    std::vector<uint8_t> packet(4 + normalMessage.data.size());
                    memcpy(packet.data(), &streamIndex, sizeof(int));
                    memcpy(packet.data() + 4, normalMessage.data.data(), normalMessage.data.size());
                    
                    // Record which stream is used
                    {
                        std::lock_guard<std::mutex> lock(g_timing_mutex);
                        auto it = g_message_timings.find(normalMessage.sequenceNumber);
                        if (it != g_message_timings.end()) {
                            it->second.streamIndex = streamIndex;
                        }
                    }
                    
                    // Send data
                    ssize_t bytesSent = send(g_dccpSocket, packet.data(), packet.size(), 0);
                    
                    if (bytesSent < 0) {
                        logError("Failed to send message: " + std::string(strerror(errno)));
                        
                        // Check if connection is lost
                        if (errno == ECONNRESET || errno == EPIPE) {
                            logError("Connection lost");
                            connectionClosed = true;
                            cv.notify_all();
                            break;
                        }
                    } else {
                        // Update metrics
                        g_metrics.messagesSent++;
                        g_metrics.messagesSentPerStream[streamIndex]++;
                        g_metrics.totalBytesSent += bytesSent;
                        
                        // Log less frequently to reduce console spam
                        if (g_metrics.messagesSent % g_logConfig.statsInterval == 0) {
                            logInfo("Sent message #" + std::to_string(normalMessage.sequenceNumber) + 
                                   " on stream " + std::to_string(streamIndex) + 
                                   " (Queue size: " + std::to_string(g_message_queue.size()) + ")");
                            
                            // Only print full metrics at larger intervals
                            if (g_metrics.messagesSent % (g_logConfig.statsInterval * 10) == 0) {
                                g_metrics.printMetrics();
                            }
                        }
                    }
                }
            }
            
            // Small delay to prevent CPU hogging
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        logInfo("Real-time data thread stopping");
    }).detach();
}

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    static int signalCount = 0;
    
    logInfo("Received signal " + std::to_string(signal) + ", initiating graceful shutdown...");
    shutdownRequested = true;
    connectionClosed = true;
    g_sendState.shutdownInitiated = true;
    cv.notify_all();
    g_sendState.cv.notify_all();
    g_queue_cv.notify_all();
    
    // Force exit on third signal
    if (++signalCount >= 3) {
        logError("Forcing exit after multiple termination signals");
        _exit(1);  // Force exit
    }
    
    // Perform immediate cleanup if not already done
    if (!cleanupComplete && g_dccpSocket >= 0) {
        close(g_dccpSocket);
        g_dccpSocket = -1;
    }
}

// Debug helper function to print bytes in hex
void dumpHexData(const uint8_t* data, uint32_t length) {
    try {
        if (!data || length == 0) {
            logDebug("  No data to display");
            return;
        }
        
        std::stringstream ss;
        ss << "  HEX: ";
        for (uint32_t i = 0; i < std::min(length, uint32_t(32)); i++) {
            ss << std::setw(2) << std::setfill('0') << std::hex << (int)data[i] << " ";
            if (i % 16 == 15) ss << "\n       ";
        }
        if (length > 32) ss << "... (" << std::dec << length << " bytes total)";
        
        logDebug(ss.str());
    } catch (const std::exception& e) {
        logError("Error in dumpHexData: " + std::string(e.what()));
    }
}

// Cleanup resources
void CleanupResources() {
    logInfo("Cleaning up resources...");
    
    // Indicate we're starting cleanup
    cleanupComplete = false;
    
    // Close socket if open
    if (g_dccpSocket >= 0) {
        logInfo("Closing DCCP socket");
        close(g_dccpSocket);
        g_dccpSocket = -1;
    }
    
    // Shutdown ROS if initialized
    if (g_ros_initialized) {
        ShutdownROS();
    }
    
    // Cleanup complete
    cleanupComplete = true;
    logInfo("Client stopped");
}

// Function to handle command line arguments for logging configuration
void parseCommandLineArgs(int argc, char* argv[], const char*& serverName) {
    // Set server name from command line if provided (first argument)
    serverName = DEFAULT_SERVER_NAME;
    if (argc > 1) {
        serverName = argv[1];
    }
    
    // Process other arguments
    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--verbose" || arg == "-v") {
            g_logConfig.level = LOG_DEBUG;
            logInfo("Verbose logging enabled");
        }
        else if (arg == "--quiet" || arg == "-q") {
            g_logConfig.level = LOG_WARNING;
            logInfo("Quiet mode enabled (warnings and errors only)");
        }
        else if (arg == "--no-color") {
            g_logConfig.colorOutput = false;
        }
        else if (arg == "--no-timestamp") {
            g_logConfig.showTimestamps = false;
        }
        else if (arg.find("--stats-interval=") == 0) {
            std::string value = arg.substr(16);
            try {
                g_logConfig.statsInterval = std::stoi(value);
                logInfo("Statistics interval set to " + value + " messages");
            } catch (...) {
                logWarning("Invalid stats interval: " + value);
            }
        }
        else if (arg == "--enable-stream-logging") {
            g_logConfig.enableStreamLogging = true;
        }
        else if (arg == "--enable-data-logging") {
            g_logConfig.enableDataLogging = true;
        }
        else if (arg.find("--rate=") == 0) {
            std::string value = arg.substr(7);
            try {
                int rate = std::stoi(value);
                if (rate >= MIN_MESSAGE_RATE && rate <= MAX_MESSAGE_RATE) {
                    g_currentMessageRate = rate;
                    g_messageInterval = std::chrono::microseconds(1000000 / g_currentMessageRate);
                    logInfo("Initial message rate set to " + value + " Hz");
                } else {
                    logWarning("Rate must be between " + std::to_string(MIN_MESSAGE_RATE) + 
                              " and " + std::to_string(MAX_MESSAGE_RATE) + " Hz. Using default.");
                }
            } catch (...) {
                logWarning("Invalid rate value: " + value);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    g_metrics.startTime = std::chrono::steady_clock::now();
    
    // Initialize logging at INFO level by default
    g_logConfig.level = LOG_INFO;
    g_logConfig.statsInterval = 1000;  // Show stats every 1000 messages
    g_logConfig.colorOutput = true;
    g_logConfig.showTimestamps = true;
    g_logConfig.enableRttLogging = true;
    g_logConfig.enableDataLogging = false;
    g_logConfig.enableStreamLogging = false;
    
    logInfo("Starting DCCP Haptic Data Client...");
    
    // Register signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);
    
    // Set server name from command line if provided
    const char* ServerName = DEFAULT_SERVER_NAME;
    parseCommandLineArgs(argc, argv, ServerName);
    
    logInfo("Using server address: " + std::string(ServerName));

    try {
        // Always initialize ROS
        if (!InitializeROS()) {
            logError("Failed to initialize ROS for real-time data");
            return 1;
        }
        
        // Setup DCCP socket
        if (!setupDCCPSocket(ServerName, ServerPort)) {
            logError("Failed to setup DCCP connection");
            return 1;
        }
        
        // Start real-time data thread
        StartRealTimeDataThread();
        
        logInfo("Connection started to " + std::string(ServerName) + ":" + std::to_string(ServerPort));
        logInfo("Streaming haptic data. Waiting for completion...");
        
        // Wait for connection to close or timeout
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            // Use longer timeout for surgical applications
            auto timeout = std::chrono::hours(2);
                
            if (!cv.wait_for(lock, timeout, []{ 
                return connectionClosed.load() || shutdownRequested.load(); 
            })) {
                logInfo("Forcing connection closure due to timeout");
                shutdownRequested = true;
                g_sendState.shutdownInitiated = true;
                if (g_dccpSocket >= 0) {
                    close(g_dccpSocket);
                    g_dccpSocket = -1;
                }
            }
        }
        
        // Print final metrics
        g_metrics.printMetrics();
        
        // Cleanup resources
        CleanupResources();
        
        return 0;
    } catch (const std::exception& e) {
        logError("Unhandled exception in main: " + std::string(e.what()));
        CleanupResources();
        return 1;
    } catch (...) {
        logError("Unknown unhandled exception in main");
        CleanupResources();
        return 1;
    }
}