#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <atomic>
#include <memory>
#include <csignal>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>
#include <linux/dccp.h>
#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
#include <omni_msgs/msg/omni_state.hpp>

// Haptic message priority
enum HapticMessagePriority {
    NORMAL = 0,
    HIGH = 1,
    EMERGENCY = 2
};

// Logging levels
enum LogLevel {
    LOG_ERROR = 0,
    LOG_WARNING = 1,
    LOG_INFO = 2,
    LOG_DEBUG = 3
};

// Global logging configuration
struct LogConfig {
    LogLevel level = LOG_INFO;
    bool colorOutput = true;
    bool showTimestamps = true;
    unsigned int statsInterval = 1000;
    bool enableRttLogging = true;
    bool enableDataLogging = false;
    bool enableStreamLogging = false;
} g_logConfig;

// Helper function for colored console output
void logMessage(LogLevel level, const std::string& message) {
    static std::mutex logMutex;
    static auto startTime = std::chrono::steady_clock::now();
    
    if (level > g_logConfig.level) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(logMutex);
    
    std::string prefix;
    std::string colorCode;
    std::string resetCode = g_logConfig.colorOutput ? "\033[0m" : "";
    
    std::string timestamp;
    if (g_logConfig.showTimestamps) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - startTime).count();
        std::stringstream ss;
        ss << "[" << std::fixed << std::setprecision(3) << elapsed << "s] ";
        timestamp = ss.str();
    }
    
    switch (level) {
    case LOG_ERROR:
        prefix = "[ERROR] ";
        colorCode = g_logConfig.colorOutput ? "\033[1;31m" : "";
        break;
    case LOG_WARNING:
        prefix = "[WARN]  ";
        colorCode = g_logConfig.colorOutput ? "\033[1;33m" : "";
        break;
    case LOG_INFO:
        prefix = "[INFO]  ";
        colorCode = g_logConfig.colorOutput ? "\033[1;36m" : "";
        break;
    case LOG_DEBUG:
        prefix = "[DEBUG] ";
        colorCode = g_logConfig.colorOutput ? "\033[1;35m" : "";
        break;
    }
    
    std::cout << colorCode << timestamp << prefix << message << resetCode << std::endl;
}

void logError(const std::string& message) { logMessage(LOG_ERROR, message); }
void logWarning(const std::string& message) { logMessage(LOG_WARNING, message); }
void logInfo(const std::string& message) { logMessage(LOG_INFO, message); }
void logDebug(const std::string& message) { logMessage(LOG_DEBUG, message); }

// Global synchronization primitives
std::mutex mtx;
std::condition_variable cv;
std::atomic<bool> connectionClosed{false};
std::atomic<bool> shutdownRequested{false};
std::atomic<bool> cleanupComplete{false};

// Client Configuration
const uint16_t ServerPort = 4433;
const char* DEFAULT_SERVER_NAME = "localhost";
const int DCCP_SERVICE_CODE = 42;
const int MAX_DCCP_PACKET_SIZE = 1400;
const int NUM_STREAMS = 4;

// DCCP socket
int g_dccpSocket = -1;
struct sockaddr_in g_serverAddr;
const size_t MAX_QUEUE_SIZE = 2000;

// Global structures for tracking message timing
struct MessageTiming {
    std::chrono::steady_clock::time_point sendTime;
    bool responseReceived = false;
    double rttMs = 0;
    int streamIndex = -1;
};

std::mutex g_timing_mutex;
std::unordered_map<uint64_t, MessageTiming> g_message_timings;

// Adaptive rate control
std::atomic<int> g_currentMessageRate{1000};
const int MIN_MESSAGE_RATE = 200;
const int MAX_MESSAGE_RATE = 1000;
std::chrono::microseconds g_messageInterval(1000000 / g_currentMessageRate);

// Enhanced Network performance metrics (matching QUIC style)
struct NetworkMetrics {
    // Latency metrics
    std::vector<double> latencies;
    double minLatency = std::numeric_limits<double>::max();
    double maxLatency = 0;
    double avgLatency = 0;
    double latencySum = 0;
    std::mutex latency_mutex;
    
    // Throughput metrics
    std::atomic<uint64_t> totalBytesSent{0};
    std::atomic<uint64_t> totalBytesReceived{0};
    std::chrono::steady_clock::time_point startTime;
    
    // Packet metrics
    std::atomic<uint64_t> messagesSent{0};
    std::atomic<uint64_t> messagesReceived{0};
    std::atomic<uint64_t> messagesLost{0};
    std::atomic<uint64_t> messagesDuplicated{0};
    std::atomic<uint64_t> messagesDropped{0};
    
    // Stream-specific metrics
    std::atomic<uint64_t> messagesSentPerStream[NUM_STREAMS]{0};
    std::atomic<uint64_t> messagesReceivedPerStream[NUM_STREAMS]{0};
    
    // Real-time measurement
    std::atomic<uint64_t> currentThroughputBps{0};
    std::atomic<double> currentLatencyMs{0};
    std::atomic<double> avgJitterMs{0};
    double cumulativeJitter = 0;
    double lastLatencyMs = 0;
    
    // Priority tracking
    std::atomic<uint64_t> emergencyMessages{0};
    std::atomic<uint64_t> highPriorityMessages{0};
    std::atomic<uint64_t> normalPriorityMessages{0};
    
    NetworkMetrics() {
        startTime = std::chrono::steady_clock::now();
    }
    
    void addLatencySample(double latencyMs, int /* streamIndex */ = -1) {
        std::lock_guard<std::mutex> lock(latency_mutex);
        
        if (latencies.size() >= 1000) {
            latencySum -= latencies[0];
            latencies.erase(latencies.begin());
        }
        
        latencies.push_back(latencyMs);
        latencySum += latencyMs;
        minLatency = std::min(minLatency, latencyMs);
        maxLatency = std::max(maxLatency, latencyMs);
        
        avgLatency = latencySum / latencies.size();
        
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
            double totalThroughputBps = (totalBytesSent * 8) / seconds;
            currentThroughputBps = static_cast<uint64_t>(totalThroughputBps);
        }
    }
    
    // QUIC-style essential metrics
    void printEssentialMetrics() {
        updateThroughput();
        std::stringstream ss;
        ss << "STATS | ";
        
        auto now = std::chrono::steady_clock::now();
        double totalTimeSeconds = std::chrono::duration<double>(now - startTime).count();
        
        // Message throughput
        double messageRate = messagesSent / totalTimeSeconds;
        ss << "Msg: " << messagesSent.load() << " (" << std::fixed << std::setprecision(1) 
           << messageRate << "/s) | ";
        
        // Success rate
        if (messagesSent > 0) {
            double successRate = (messagesReceived * 100.0) / messagesSent;
            ss << "Success: " << std::fixed << std::setprecision(2) << successRate << "% | ";
        }
        
        // Data throughput
        double dataRateMbps = (totalBytesSent * 8) / (totalTimeSeconds * 1000000);
        ss << "Rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps | ";
        
        // Latency stats if available
        if (!latencies.empty()) {
            ss << "RTT: " << std::fixed << std::setprecision(2) 
               << minLatency << "/" << avgLatency << "/" 
               << maxLatency << " ms | ";
            ss << "Jitter: " << std::fixed << std::setprecision(3) << avgJitterMs << " ms";
        }
        
        logInfo(ss.str());
    }
    
    // QUIC-style compact status
    void printCompactStatus() {
        if (messagesSent == 0) return;
        
        std::stringstream ss;
        
        auto now = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration<double>(now - startTime).count();
        
        double msgRate = messagesSent / seconds;
        
        double receiveRate = 0;
        if (messagesSent > 0) {
            receiveRate = (messagesReceived * 100.0) / messagesSent;
        }
        
        std::string latencyStr = "N/A";
        if (!latencies.empty()) {
            std::stringstream latSs;
            latSs << std::fixed << std::setprecision(2) << currentLatencyMs.load();
            latencyStr = latSs.str();
        }
        
        ss << "[Status] Msgs: " << messagesSent.load() << " @ " 
           << std::fixed << std::setprecision(0) << msgRate << "/s | "
           << "Success: " << std::fixed << std::setprecision(1) << receiveRate << "% | "
           << "Latency: " << latencyStr << " ms";
        
        logInfo(ss.str());
    }
    
    // QUIC-style full metrics
    void printMetrics() {
        updateThroughput();
        
        // Define ANSI color codes
        const std::string CYAN = g_logConfig.colorOutput ? "\033[1;36m" : "";
        const std::string GREEN = g_logConfig.colorOutput ? "\033[1;32m" : "";
        const std::string YELLOW = g_logConfig.colorOutput ? "\033[1;33m" : "";
        const std::string RED = g_logConfig.colorOutput ? "\033[1;31m" : "";
        const std::string RESET = g_logConfig.colorOutput ? "\033[0m" : "";

        std::cout << "\n" << CYAN << "===== NETWORK METRICS =====" << RESET << std::endl;
        std::cout << CYAN << "Messages sent: " << messagesSent << RESET << std::endl;
        std::cout << CYAN << "Messages received: " << messagesReceived << RESET << std::endl;
        
        // Calculate packet loss rate
        uint64_t sentMsgs = messagesSent.load();
        uint64_t receivedMsgs = messagesReceived.load();
        
        if (sentMsgs > 0) {
            double lossRate = ((sentMsgs - receivedMsgs) * 100.0) / sentMsgs;
            std::cout << CYAN << "Messages lost: " << (sentMsgs - receivedMsgs) << " (" << std::fixed << std::setprecision(2) << lossRate << "%)" << RESET << std::endl;
        }
        
        if (messagesDuplicated > 0) {
            std::cout << YELLOW << "Duplicated messages: " << messagesDuplicated.load() << RESET << std::endl;
        }
        
        if (messagesDropped > 0) {
            std::cout << YELLOW << "Dropped due to queue overflow: " << messagesDropped.load() << RESET << std::endl;
        }
        
        std::cout << CYAN << "Total data sent: " << totalBytesSent.load() << " bytes" << RESET << std::endl;
        std::cout << CYAN << "Total data received: " << totalBytesReceived.load() << " bytes" << RESET << std::endl;
        std::cout << CYAN << "Throughput: " << (currentThroughputBps / 1000.0) << " Kbps" << RESET << std::endl;
        
        // Priority statistics
        std::cout << CYAN << "Message priorities: "
                  << RED << "Emergency: " << emergencyMessages << " | "
                  << YELLOW << "High: " << highPriorityMessages << " | "
                  << GREEN << "Normal: " << normalPriorityMessages << RESET << std::endl;
        
        // Per-stream statistics
        std::cout << CYAN << "Per-Stream Statistics:" << RESET << std::endl;
        for (int i = 0; i < NUM_STREAMS; i++) {
            std::cout << CYAN << "  Stream " << i << ": Sent " << messagesSentPerStream[i] 
                      << ", Received " << messagesReceivedPerStream[i] << RESET << std::endl;
        }
        
        std::lock_guard<std::mutex> lock(latency_mutex);
        if (!latencies.empty()) {
            std::cout << CYAN << "Latency (min/avg/max): " << std::fixed << std::setprecision(3) 
                      << minLatency << " / " << avgLatency << " / " << maxLatency << " ms" << RESET << std::endl;
            std::cout << CYAN << "Average jitter: " << std::fixed << std::setprecision(5) << avgJitterMs << " ms" << RESET << std::endl;
        }
        
        std::cout << CYAN << "Current message rate: " << g_currentMessageRate << " Hz" << RESET << std::endl;
        std::cout << CYAN << "============================" << RESET << std::endl;
    }
};

NetworkMetrics g_metrics;

// Message structure with timestamp and priority
struct MessageWithTimestamp {
    std::vector<uint8_t> data;
    std::chrono::steady_clock::time_point sendTime;
    uint64_t sequenceNumber;
    HapticMessagePriority priority = NORMAL;
};

std::mutex g_data_mutex;
std::queue<MessageWithTimestamp> g_message_queue;
std::condition_variable g_queue_cv;
std::atomic<uint64_t> g_sequence_number{0};

// Global ROS2 variables
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr g_subscription = nullptr;
std::atomic<bool> g_ros_initialized{false};
std::mutex g_ros_mutex;

// DCCP socket setup
bool setupDCCPSocket(const std::string& serverName, uint16_t port) {
    logInfo("Setting up DCCP socket to " + serverName + ":" + std::to_string(port));

    g_dccpSocket = socket(AF_INET, SOCK_DCCP, IPPROTO_DCCP);
    if (g_dccpSocket < 0) {
        logError("Failed to create DCCP socket: " + std::string(strerror(errno)));
        return false;
    }

    int service_code = htonl(DCCP_SERVICE_CODE);
    if (setsockopt(g_dccpSocket, SOL_DCCP, DCCP_SOCKOPT_SERVICE, &service_code, sizeof(service_code)) < 0) {
        logError("Failed to set DCCP service code: " + std::string(strerror(errno)));
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }

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

    logInfo("Connecting to DCCP server at " + serverName + ":" + std::to_string(port));
    if (connect(g_dccpSocket, (struct sockaddr*)&g_serverAddr, sizeof(g_serverAddr)) < 0) {
        logError("Failed to connect to server: " + std::string(strerror(errno)));
        close(g_dccpSocket);
        g_dccpSocket = -1;
        return false;
    }

    logInfo("Connected to DCCP server successfully");
    return true;
}

// Enhanced callback for omni_state topic
void omniStateCallback(const std::shared_ptr<const omni_msgs::msg::OmniState> msg) {
    try {
        MessageWithTimestamp message;
        message.sendTime = std::chrono::steady_clock::now();
        message.sequenceNumber = g_sequence_number++;
        
        // Determine message priority
        float forceThreshold = 20.0f;
        float force = std::sqrt(
            msg->current.x * msg->current.x + 
            msg->current.y * msg->current.y + 
            msg->current.z * msg->current.z);
            
        if (force > forceThreshold || msg->locked) {
            message.priority = EMERGENCY;
            g_metrics.emergencyMessages++;
        }
        else if (force > forceThreshold * 0.7) {
            message.priority = HIGH;
            g_metrics.highPriorityMessages++;
        } else {
            message.priority = NORMAL;
            g_metrics.normalPriorityMessages++;
        }
        
        // Create message buffer
        const size_t bufferSize = 256;
        message.data.resize(bufferSize);
        
        // Sequence number and timestamp
        memcpy(message.data.data(), &message.sequenceNumber, sizeof(uint64_t));
        uint64_t absoluteTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            message.sendTime.time_since_epoch()).count();
        memcpy(message.data.data() + sizeof(uint64_t), &absoluteTimestamp, sizeof(uint64_t));
        
        // Store timing information
        {
            std::lock_guard<std::mutex> lock(g_timing_mutex);
            g_message_timings[message.sequenceNumber] = {message.sendTime, false, 0, -1};
            
            // Cleanup old entries
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
        
        // Add haptic data
        float x = static_cast<float>(msg->pose.position.x);
        float y = static_cast<float>(msg->pose.position.y);
        float z = static_cast<float>(msg->pose.position.z);
        
        memcpy(message.data.data() + 16, &x, sizeof(float));
        memcpy(message.data.data() + 20, &y, sizeof(float));
        memcpy(message.data.data() + 24, &z, sizeof(float));
        
        // Add priority
        uint8_t priorityValue = static_cast<uint8_t>(message.priority);
        memcpy(message.data.data() + 70, &priorityValue, sizeof(uint8_t));
        
        // Add to queue
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            
            if (g_message_queue.size() >= MAX_QUEUE_SIZE) {
                g_message_queue.pop();
                g_metrics.messagesDropped++;
            }
            
            g_message_queue.push(std::move(message));
        }
        g_queue_cv.notify_one();
        
    } catch (const std::exception& e) {
        logError("Error in omni state callback: " + std::string(e.what()));
    }
}

// Initialize ROS2
bool InitializeROS() {
    std::lock_guard<std::mutex> lock(g_ros_mutex);
    
    if (g_ros_initialized) return true;
    
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        g_node = std::make_shared<rclcpp::Node>("dccp_haptic_client");
        
        g_subscription = g_node->create_subscription<omni_msgs::msg::OmniState>(
            "/phantom/state", 10, omniStateCallback);
            
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
        logInfo("ROS2 initialized");
        return true;
    } catch (const std::exception& e) {
        logError("Error initializing ROS2: " + std::string(e.what()));
        return false;
    }
}

// Read thread for receiving messages
void StartReadThread() {
    std::thread([&]() {
        logInfo("Read thread started");
        
        uint8_t buffer[MAX_DCCP_PACKET_SIZE];
        
        while (!shutdownRequested.load() && g_dccpSocket >= 0) {
            struct pollfd pfd;
            pfd.fd = g_dccpSocket;
            pfd.events = POLLIN;
            int ret = poll(&pfd, 1, 100);
            
            if (ret < 0) {
                if (errno != EINTR) {
                    logError("Poll error: " + std::string(strerror(errno)));
                    break;
                }
                continue;
            }
            
            if (ret == 0) {
                continue;
            }
            
            ssize_t bytesRead = recv(g_dccpSocket, buffer, sizeof(buffer), 0);
            
            if (bytesRead < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    logError("Error receiving data: " + std::string(strerror(errno)));
                    break;
                }
                continue;
            }
            
            if (bytesRead == 0) {
                logInfo("Connection closed by server");
                connectionClosed = true;
                cv.notify_all();
                break;
            }
            
            if (bytesRead < 20) {
                continue;
            }
            
            // Extract ACK data
            int streamIndex;
            uint64_t sequenceNumber, timestamp;
            memcpy(&streamIndex, buffer, sizeof(int));
            memcpy(&sequenceNumber, buffer + 4, sizeof(uint64_t));
            memcpy(&timestamp, buffer + 12, sizeof(uint64_t));
            
            if (streamIndex < 0 || streamIndex >= NUM_STREAMS) {
                continue;
            }
            
            auto now = std::chrono::steady_clock::now();
            
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
                    
                    g_message_timings.erase(it);
                }
            }
            
            // Print statistics at intervals
            if (g_metrics.messagesReceived % g_logConfig.statsInterval == 0) {
                g_metrics.printEssentialMetrics();
            }
        }
        
        logInfo("Read thread stopping");
    }).detach();
}

// Enhanced data sending thread
void StartRealTimeDataThread() {
    logInfo("Starting real-time data thread");
    
    StartReadThread();
    
    std::thread([&]() {
        logDebug("Message processing thread started");
        
        auto lastSendTime = std::chrono::steady_clock::now();
        auto lastStatusTime = std::chrono::steady_clock::now();
        const auto STATUS_INTERVAL = std::chrono::seconds(5);
        
        while (!shutdownRequested.load() && g_dccpSocket >= 0) {
            auto currentTime = std::chrono::steady_clock::now();
            if (currentTime - lastStatusTime > STATUS_INTERVAL) {
                g_metrics.printCompactStatus();
                lastStatusTime = currentTime;
            }
            
            {
                std::unique_lock<std::mutex> lock(g_data_mutex);
                if (g_message_queue.empty()) {
                    g_queue_cv.wait_for(lock, std::chrono::milliseconds(10), 
                        []{ return !g_message_queue.empty() || shutdownRequested.load(); });
                }
                
                if (shutdownRequested.load() || g_message_queue.empty()) {
                    continue;
                }
            }
            
            MessageWithTimestamp message;
            {
                std::unique_lock<std::mutex> lock(g_data_mutex);
                if (!g_message_queue.empty()) {
                    message = g_message_queue.front();
                    g_message_queue.pop();
                } else {
                    continue;
                }
            }
            
            auto timeSinceLastSend = currentTime - lastSendTime;
            
            // Apply rate limiting
            if (message.priority == NORMAL && timeSinceLastSend < g_messageInterval) {
                std::this_thread::sleep_for(g_messageInterval - timeSinceLastSend);
            }
            lastSendTime = std::chrono::steady_clock::now();
            
            // Select stream (round-robin for normal messages)
            static int roundRobinIndex = 0;
            int streamIndex = (roundRobinIndex++ % NUM_STREAMS);
            
            if (g_dccpSocket >= 0) {
                // Prepend stream index
                std::vector<uint8_t> packet(4 + message.data.size());
                memcpy(packet.data(), &streamIndex, sizeof(int));
                memcpy(packet.data() + 4, message.data.data(), message.data.size());
                
                // Record stream used
                {
                    std::lock_guard<std::mutex> lock(g_timing_mutex);
                    auto it = g_message_timings.find(message.sequenceNumber);
                    if (it != g_message_timings.end()) {
                        it->second.streamIndex = streamIndex;
                    }
                }
                
                ssize_t bytesSent = send(g_dccpSocket, packet.data(), packet.size(), 0);
                
                if (bytesSent < 0) {
                    if (errno == ECONNRESET || errno == EPIPE) {
                        logError("Connection lost");
                        connectionClosed = true;
                        cv.notify_all();
                        break;
                    }
                } else {
                    g_metrics.messagesSent++;
                    g_metrics.messagesSentPerStream[streamIndex]++;
                    g_metrics.totalBytesSent += bytesSent;
                    
                    // Print full metrics periodically
                    if (g_metrics.messagesSent % (g_logConfig.statsInterval * 10) == 0) {
                        g_metrics.printMetrics();
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        logInfo("Real-time data thread stopping");
    }).detach();
}

// Signal handler
void signalHandler(int signal) {
    static int signalCount = 0;
    
    logInfo("Received signal " + std::to_string(signal) + ", initiating graceful shutdown...");
    shutdownRequested = true;
    connectionClosed = true;
    cv.notify_all();
    g_queue_cv.notify_all();
    
    if (++signalCount >= 3) {
        logError("Forcing exit after multiple termination signals");
        _exit(1);
    }
    
    if (!cleanupComplete && g_dccpSocket >= 0) {
        close(g_dccpSocket);
        g_dccpSocket = -1;
    }
}

// Cleanup resources
void CleanupResources() {
    logInfo("Cleaning up resources...");
    
    cleanupComplete = false;
    
    if (g_dccpSocket >= 0) {
        logInfo("Closing DCCP socket");
        close(g_dccpSocket);
        g_dccpSocket = -1;
    }
    
    if (g_ros_initialized) {
        std::lock_guard<std::mutex> lock(g_ros_mutex);
        try {
            g_subscription.reset();
            g_node.reset();
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            g_ros_initialized = false;
            logInfo("ROS2 shutdown complete");
        } catch (const std::exception& e) {
            logError("Error shutting down ROS2: " + std::string(e.what()));
        }
    }
    
    cleanupComplete = true;
    logInfo("Client stopped");
}

int main(int argc, char* argv[]) {
    g_metrics.startTime = std::chrono::steady_clock::now();
    
    logInfo("Starting DCCP Haptic Data Client...");
    
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    const char* ServerName = DEFAULT_SERVER_NAME;
    if (argc > 1) {
        ServerName = argv[1];
    }
    
    logInfo("Using server address: " + std::string(ServerName));

    try {
        if (!InitializeROS()) {
            logError("Failed to initialize ROS");
            return 1;
        }
        
        if (!setupDCCPSocket(ServerName, ServerPort)) {
            logError("Failed to setup DCCP connection");
            return 1;
        }
        
        StartRealTimeDataThread();
        
        logInfo("Connection started to " + std::string(ServerName) + ":" + std::to_string(ServerPort));
        logInfo("Streaming haptic data. Waiting for completion...");
        
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            auto timeout = std::chrono::hours(2);
                
            if (!cv.wait_for(lock, timeout, []{ 
                return connectionClosed.load() || shutdownRequested.load(); 
            })) {
                logInfo("Forcing connection closure due to timeout");
                shutdownRequested = true;
                if (g_dccpSocket >= 0) {
                    close(g_dccpSocket);
                    g_dccpSocket = -1;
                }
            }
        }
        
        // Print final metrics
        g_metrics.printMetrics();
        
        CleanupResources();
        
        return 0;
    } catch (const std::exception& e) {
        logError("Unhandled exception in main: " + std::string(e.what()));
        CleanupResources();
        return 1;
    }
}