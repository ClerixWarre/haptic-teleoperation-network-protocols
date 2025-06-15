#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <condition_variable>
#include <queue>
#include <future>
#include <random>  // Add missing include
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <ifaddrs.h>
#include <net/if.h>
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
    bool enableDataLogging = false;
    bool enableProtocolAnalysis = false;
    double artificialProcessingDelayMs = 0.0;
    bool useRandomProcessingDelay = false;
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
std::mutex g_stream_mutex;
std::mutex g_output_mutex;
std::condition_variable g_shutdown_cv;

// Server Configuration
const uint16_t ServerPort = 4433;
const int DCCP_SERVICE_CODE = 42;
const int MAX_DCCP_PACKET_SIZE = 1400;
const int NUM_STREAMS = 4;

// Global flags
std::atomic<bool> g_shutdown_requested{false};
int g_server_socket = -1;
int g_client_socket = -1;

// Enhanced message structure
struct HapticMessage {
    uint64_t sequenceNumber;
    uint64_t timestamp;
    float posX, posY, posZ;
    HapticMessagePriority priority;
    int streamIndex;
    
    HapticMessage() : 
        sequenceNumber(0), timestamp(0),
        posX(0.0f), posY(0.0f), posZ(0.0f),
        priority(NORMAL), streamIndex(0) {}
        
    bool parseFromBuffer(const uint8_t* buffer, uint32_t length) {
        if (!buffer || length < 16) {
            return false;
        }
        
        try {
            // Extract stream index (first 4 bytes from packet header)
            memcpy(&streamIndex, buffer, sizeof(int));
            
            // Extract sequence number and timestamp from message payload
            memcpy(&sequenceNumber, buffer + 4, sizeof(uint64_t));
            memcpy(&timestamp, buffer + 12, sizeof(uint64_t));
            
            // Extract position if available
            if (length >= 32) {
                memcpy(&posX, buffer + 20, sizeof(float));
                memcpy(&posY, buffer + 24, sizeof(float));
                memcpy(&posZ, buffer + 28, sizeof(float));
            }
            
            // Extract priority if available
            if (length >= 75) {
                uint8_t priorityValue;
                memcpy(&priorityValue, buffer + 74, sizeof(uint8_t));
                priority = static_cast<HapticMessagePriority>(priorityValue);
            }
            
            return true;
        } catch (...) {
            return false;
        }
    }
};

// Enhanced network stats (QUIC-style)
struct NetworkStats {
    std::atomic<uint64_t> messagesReceived{0};
    std::atomic<uint64_t> bytesReceived{0};
    std::atomic<uint64_t> messagesSent{0};
    std::atomic<uint64_t> bytesSent{0};
    std::chrono::steady_clock::time_point startTime;
    
    // Priority tracking
    std::atomic<uint64_t> emergencyMessages{0};
    std::atomic<uint64_t> highPriorityMessages{0};
    std::atomic<uint64_t> normalPriorityMessages{0};
    
    // Messages per stream
    std::atomic<uint64_t> messagesReceivedPerStream[NUM_STREAMS]{0};
    std::atomic<uint64_t> messagesSentPerStream[NUM_STREAMS]{0};
    
    NetworkStats() {
        startTime = std::chrono::steady_clock::now();
    }

    void update(uint64_t bytes, HapticMessagePriority priority = NORMAL, int streamIndex = 0) {
        messagesReceived++;
        bytesReceived += bytes;
        
        // Update stream stats
        if (streamIndex >= 0 && streamIndex < NUM_STREAMS) {
            messagesReceivedPerStream[streamIndex]++;
        }
        
        // Update priority counters
        switch (priority) {
            case EMERGENCY:
                emergencyMessages++;
                break;
            case HIGH:
                highPriorityMessages++;
                break;
            case NORMAL:
                normalPriorityMessages++;
                break;
        }

        // Print statistics periodically
        if (messagesReceived % g_logConfig.statsInterval == 0) {
            printCompactStatus();
        }
    }
    
    void updateSent(uint64_t bytes, int streamIndex = 0) {
        messagesSent++;
        bytesSent += bytes;
        
        if (streamIndex >= 0 && streamIndex < NUM_STREAMS) {
            messagesSentPerStream[streamIndex]++;
        }
    }

    void printCompactStatus() {
        auto now = std::chrono::steady_clock::now();
        double totalTimeSeconds = std::chrono::duration<double>(now - startTime).count();
        double messageRate = messagesReceived / totalTimeSeconds;
        double dataRateMbps = (bytesReceived * 8) / (totalTimeSeconds * 1000000);
        
        const std::string CYAN = g_logConfig.colorOutput ? "\033[1;36m" : "";
        const std::string RESET = g_logConfig.colorOutput ? "\033[0m" : "";
        
        std::stringstream ss;
        ss << CYAN << "STATS | Msg: " << messagesReceived << " (" 
           << std::fixed << std::setprecision(1) << messageRate << "/s) | "
           << "Rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps | "
           << "Emergency: " << emergencyMessages << " | "
           << "High: " << highPriorityMessages << " | "
           << "Normal: " << normalPriorityMessages << RESET;
        
        logInfo(ss.str());
    }

    void printStats() {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        auto now = std::chrono::steady_clock::now();
        double totalTimeSeconds = std::chrono::duration<double>(now - startTime).count();
        double messageRate = messagesReceived / totalTimeSeconds;
        double dataRateMbps = (bytesReceived * 8) / (totalTimeSeconds * 1000000);

        const std::string CYAN = g_logConfig.colorOutput ? "\033[1;36m" : "";
        const std::string GREEN = g_logConfig.colorOutput ? "\033[1;32m" : "";
        const std::string YELLOW = g_logConfig.colorOutput ? "\033[1;33m" : "";
        const std::string RED = g_logConfig.colorOutput ? "\033[1;31m" : "";
        const std::string RESET = g_logConfig.colorOutput ? "\033[0m" : "";

        std::cout << "\n" << CYAN << "===== NETWORK STATISTICS =====" << RESET << std::endl;
        std::cout << CYAN << "Messages received: " << messagesReceived << RESET << std::endl;
        std::cout << CYAN << "Total bytes received: " << bytesReceived << RESET << std::endl;
        std::cout << CYAN << "Messages per second: " << std::fixed << std::setprecision(1) << messageRate << RESET << std::endl;
        std::cout << CYAN << "Data rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps" << RESET << std::endl;
        std::cout << CYAN << "ACKs sent: " << messagesSent << " (" << bytesSent << " bytes)" << RESET << std::endl;
        
        std::cout << CYAN << "Message priorities: "
                  << RED << "Emergency: " << emergencyMessages << " | "
                  << YELLOW << "High: " << highPriorityMessages << " | "
                  << GREEN << "Normal: " << normalPriorityMessages << RESET << std::endl;
        
        std::cout << CYAN << "Per-Stream Statistics:" << RESET << std::endl;
        for (int i = 0; i < NUM_STREAMS; i++) {
            std::cout << CYAN << "  Stream " << i << ": Received " << messagesReceivedPerStream[i] 
                      << ", ACKs sent " << messagesSentPerStream[i] << RESET << std::endl;
        }
        
        std::cout << CYAN << "Running time: " << std::fixed << std::setprecision(1) << totalTimeSeconds << " seconds" << RESET << std::endl;
        std::cout << CYAN << "=============================" << RESET << std::endl;
    }
};

NetworkStats g_stats;

// Global ROS2 variables
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr g_publisher = nullptr;
std::atomic<bool> g_ros_initialized{false};
std::mutex g_ros_mutex;

// Setup DCCP server socket
bool setupDCCPServer(uint16_t port) {
    logInfo("Setting up DCCP server on port " + std::to_string(port));
    
    g_server_socket = socket(AF_INET, SOCK_DCCP, IPPROTO_DCCP);
    if (g_server_socket < 0) {
        logError("Failed to create DCCP socket: " + std::string(strerror(errno)));
        return false;
    }
    
    int service_code = htonl(DCCP_SERVICE_CODE);
    if (setsockopt(g_server_socket, SOL_DCCP, DCCP_SOCKOPT_SERVICE, &service_code, sizeof(service_code)) < 0) {
        logError("Failed to set DCCP service code: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    int reuseaddr = 1;
    if (setsockopt(g_server_socket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) < 0) {
        logError("Failed to set SO_REUSEADDR: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port);
    
    if (bind(g_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        logError("Failed to bind socket: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    if (listen(g_server_socket, 5) < 0) {
        logError("Failed to listen on socket: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    logInfo("DCCP server setup complete on port " + std::to_string(port));
    return true;
}

bool acceptClient() {
    logInfo("Waiting for client connection...");
    
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    
    g_client_socket = accept(g_server_socket, (struct sockaddr*)&client_addr, &client_addr_len);
    if (g_client_socket < 0) {
        logError("Failed to accept client connection: " + std::string(strerror(errno)));
        return false;
    }
    
    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    logInfo("Client connected from: " + std::string(client_ip) + ":" + std::to_string(ntohs(client_addr.sin_port)));
    
    return true;
}

// Enhanced message processing
void ProcessHapticData(const uint8_t* buffer, uint32_t length, uint64_t globalMsgCount) {
    if (!buffer || length < 16) {
        return;
    }

    if (g_shutdown_requested.load()) {
        return;
    }

    try {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Add artificial processing delay if configured (fixed random generation)
        if (g_logConfig.artificialProcessingDelayMs > 0.0) {
            double delayMs = g_logConfig.artificialProcessingDelayMs;
            if (g_logConfig.useRandomProcessingDelay) {
                // Simple random without complex includes
                delayMs *= (rand() % 1000) / 1000.0;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<long>(delayMs * 1000)));
        }
        
        HapticMessage msg;
        bool parseSuccess = msg.parseFromBuffer(buffer, length);
        
        if (!parseSuccess) {
            logWarning("Failed to parse message data");
            return;
        }
        
        double processingLatencyMs = std::chrono::duration<double, std::micro>(
            std::chrono::high_resolution_clock::now() - startTime).count() / 1000.0;
            
        if (msg.priority == EMERGENCY) {
            logInfo("EMERGENCY MESSAGE #" + std::to_string(msg.sequenceNumber) + 
                   " processing time: " + std::to_string(processingLatencyMs) + " ms");
        } else if (msg.priority == HIGH) {
            logInfo("HIGH PRIORITY MESSAGE #" + std::to_string(msg.sequenceNumber) + 
                   " processing time: " + std::to_string(processingLatencyMs) + " ms");
        }
            
        g_stats.update(length, msg.priority, msg.streamIndex);
        
        if (globalMsgCount % 1000 == 0) {
            logDebug("Message #" + std::to_string(msg.sequenceNumber) + 
                   " position: " + std::to_string(msg.posX) + ", " + 
                   std::to_string(msg.posY) + ", " + std::to_string(msg.posZ));
        }
        
        // Publish to ROS if initialized
        if (g_ros_initialized && g_publisher && g_node && rclcpp::ok() && !g_shutdown_requested.load()) {
            try {
                auto rosMsg = std::make_shared<omni_msgs::msg::OmniState>();
                
                rosMsg->header.frame_id = "world";
                rosMsg->header.stamp = g_node->now();
                
                rosMsg->pose.position.x = msg.posX;
                rosMsg->pose.position.y = msg.posY;
                rosMsg->pose.position.z = msg.posZ;
                
                rosMsg->pose.orientation.w = 1.0;
                rosMsg->pose.orientation.x = 0.0;
                rosMsg->pose.orientation.y = 0.0;
                rosMsg->pose.orientation.z = 0.0;
                
                g_publisher->publish(*rosMsg);
                
                if (globalMsgCount % 1000 == 0) {
                    logDebug("Published to ROS: message #" + std::to_string(msg.sequenceNumber));
                }
            } catch (const std::exception& e) {
                if (globalMsgCount % 100 == 0) {
                    logWarning("Error publishing to ROS: " + std::string(e.what()));
                }
            }
        }
        
        // Send acknowledgment
        if (g_client_socket >= 0) {
            uint8_t ackBuffer[20];
            memcpy(ackBuffer, &msg.streamIndex, sizeof(int));
            memcpy(ackBuffer + 4, &msg.sequenceNumber, sizeof(uint64_t));
            memcpy(ackBuffer + 12, &msg.timestamp, sizeof(uint64_t));
            
            ssize_t bytesSent = send(g_client_socket, ackBuffer, sizeof(ackBuffer), 0);
            
            if (bytesSent > 0) {
                g_stats.updateSent(bytesSent, msg.streamIndex);
            } else {
                logError("Failed to send ACK: " + std::string(strerror(errno)));
            }
        }
    } catch (const std::exception& e) {
        logError("Error processing haptic data: " + std::string(e.what()));
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
        
        g_node = std::make_shared<rclcpp::Node>("dccp_haptic_server");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        g_publisher = g_node->create_publisher<omni_msgs::msg::OmniState>(
            "/phantom/remote_state", qos);
            
        std::thread([&]() {
            logInfo("ROS2 spin thread started");
            while (!g_shutdown_requested && rclcpp::ok()) {
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

// Shutdown ROS2
void ShutdownROS() {
    std::lock_guard<std::mutex> lock(g_ros_mutex);
    
    try {
        logInfo("Beginning ROS2 shutdown sequence...");
        g_shutdown_requested = true;
        
        if (g_ros_initialized) {
            if (g_publisher) {
                g_publisher.reset();
            }
            
            if (g_node) {
                g_node.reset();
            }
            
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
            
            g_ros_initialized = false;
            logInfo("ROS2 shutdown complete");
        }
    } catch (const std::exception& e) {
        logError("Error shutting down ROS2: " + std::string(e.what()));
    }
}

// Signal handler
void signalHandler(int signal) {
    static int signalCount = 0;
    
    logInfo("Received signal " + std::to_string(signal) + ", initiating graceful shutdown...");
    g_shutdown_requested = true;
    g_shutdown_cv.notify_all();
    
    if (++signalCount >= 3) {
        std::cerr << "Forcing exit after multiple termination signals\n";
        _exit(1);
    }
}

// Main server thread
void runServer() {
    logInfo("Starting DCCP server main loop");
    
    uint64_t globalMsgCount = 0;
    uint8_t buffer[MAX_DCCP_PACKET_SIZE];
    
    while (!g_shutdown_requested.load() && g_client_socket >= 0) {
        struct pollfd pfd;
        pfd.fd = g_client_socket;
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
        
        ssize_t bytesRead = recv(g_client_socket, buffer, sizeof(buffer), 0);
        
        if (bytesRead < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                logError("Error receiving data: " + std::string(strerror(errno)));
                break;
            }
            continue;
        }
        
        if (bytesRead == 0) {
            logInfo("Client disconnected");
            break;
        }
        
        globalMsgCount++;
        ProcessHapticData(buffer, bytesRead, globalMsgCount);
    }
    
    logInfo("Server main loop exiting");
}

int main(int /* argc */, char* /* argv */[]) {  // Fix unused parameter warnings
    g_stats.startTime = std::chrono::steady_clock::now();
    
    logInfo("Starting DCCP Haptic Data Server...");
    
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    try {
        if (!setupDCCPServer(ServerPort)) {
            logError("Failed to setup DCCP server");
            return 1;
        }
        
        logInfo("DCCP server started on port " + std::to_string(ServerPort));
        
        InitializeROS();
        
        if (!acceptClient()) {
            close(g_server_socket);
            return 1;
        }
        
        logInfo("Server ready, processing messages...");
        
        std::thread serverThread(runServer);
        
        // Wait for shutdown
        std::mutex waitMtx;
        std::unique_lock<std::mutex> waitLock(waitMtx);
        g_shutdown_cv.wait(waitLock, [&]() {
            return g_shutdown_requested.load();
        });
        
        g_shutdown_requested = true;
        
        // Print final statistics
        g_stats.printStats();
        
        if (serverThread.joinable()) {
            serverThread.join();
        }
        
        logInfo("Stopping server...");
        
        if (g_client_socket >= 0) {
            close(g_client_socket);
            g_client_socket = -1;
        }
        
        if (g_server_socket >= 0) {
            close(g_server_socket);
            g_server_socket = -1;
        }
        
        ShutdownROS();
        
        logInfo("Server stopped");
        return 0;
    } catch (const std::exception& e) {
        logError("Unhandled exception in main: " + std::string(e.what()));
        ShutdownROS();
        return 1;
    }
}
