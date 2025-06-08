#include <iostream>
#include <string>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <csignal>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <random>
#include <condition_variable>
#include <queue>
#include <future>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <linux/dccp.h>
#include <sys/types.h>

// ROS2 include files
#include <rclcpp/rclcpp.hpp>
#include <omni_msgs/msg/omni_state.hpp>

// Haptic message priority - match client enum
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
    bool enableDataLogging = false;
    bool enableProtocolAnalysis = false;
    
    // Add artificial processing delay for testing (0 = no delay)
    double artificialProcessingDelayMs = 0.0;
    bool useRandomProcessingDelay = false;  // If true, uses a random value up to the value above
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
std::mutex g_stream_mutex;
std::mutex g_output_mutex;
std::condition_variable g_shutdown_cv;

// Server Configuration
const uint16_t ServerPort = 4433;
const int DCCP_SERVICE_CODE = 42; // Custom service code
const int MAX_DCCP_PACKET_SIZE = 1400; // Maximum datagram size
const int NUM_STREAMS = 4; // Virtual streams (like QUIC implementation)

// Global flags
std::atomic<bool> g_shutdown_requested{false};
int g_server_socket = -1;

// Mutex for thread safety
std::recursive_mutex g_stats_mutex;

// Thread pool for surgical robotics data processing
class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    std::atomic<bool> stop;
    std::atomic<int> active_threads;
    
public:
    ThreadPool(size_t threads) : stop(false), active_threads(0) {
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { 
                            return stop || !tasks.empty(); 
                        });
                        
                        if (stop && tasks.empty()) {
                            return;
                        }
                        
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    
                    active_threads++;
                    task();
                    active_threads--;
                }
            });
        }
    }
    
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;
        
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
        std::future<return_type> res = task->get_future();
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            
            tasks.emplace([task]() { (*task)(); });
        }
        
        condition.notify_one();
        return res;
    }
    
    int activeThreadCount() const {
        return active_threads.load();
    }
    
    size_t queueSize() {
        std::unique_lock<std::mutex> lock(queue_mutex);
        return tasks.size();
    }
    
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        
        condition.notify_all();
        
        for (std::thread &worker: workers) {
            worker.join();
        }
    }
};

// Create thread pool with optimal size for surgical applications
const size_t THREAD_POOL_SIZE = std::max(4u, std::thread::hardware_concurrency());
std::unique_ptr<ThreadPool> g_thread_pool;

// Store client information
struct ClientInfo {
    struct sockaddr_in addr;
    socklen_t addrLen;
    std::chrono::steady_clock::time_point lastActive;
    
    ClientInfo() : addrLen(sizeof(addr)) {
        memset(&addr, 0, sizeof(addr));
        lastActive = std::chrono::steady_clock::now();
    }
};

// Map of connected clients
std::mutex g_clients_mutex;
std::unordered_map<uint32_t, ClientInfo> g_clients; // Key: IP address as uint32_t

// Define haptic message structure for validation
struct HapticMessage {
    uint64_t sequenceNumber;
    uint64_t timestamp;
    float posX;
    float posY;
    float posZ;
    float quatW;
    float quatX;
    float quatY;
    float quatZ;
    float velX;
    float velY;
    float velZ;
    float currX;
    float currY;
    float currZ;
    bool locked;
    bool closeGripper;
    HapticMessagePriority priority;
    int streamIndex; // Virtual stream this message belongs to
    
    // Default constructor
    HapticMessage() : 
        sequenceNumber(0), timestamp(0),
        posX(0.0f), posY(0.0f), posZ(0.0f),
        quatW(1.0f), quatX(0.0f), quatY(0.0f), quatZ(0.0f),
        velX(0.0f), velY(0.0f), velZ(0.0f),
        currX(0.0f), currY(0.0f), currZ(0.0f),
        locked(false), closeGripper(false),
        priority(NORMAL), streamIndex(0) {}
        
    // Improved parseFromBuffer with better error checking and debugging
    bool parseFromBuffer(const uint8_t* buffer, uint32_t length) {
        // Safe guard check
        if (!buffer || length < 16) {
            logWarning("Invalid buffer or length too small: " + std::to_string(length) + " bytes");
            return false; // Not enough data for basic fields
        }
        
        // Debug output
        if (g_logConfig.enableDataLogging) {
            std::stringstream ss;
            ss << "Parsing buffer of length " << length << ":";
            for (uint32_t i = 0; i < std::min(length, uint32_t(32)); i++) {
                if (i % 8 == 0) ss << "\n  ";
                ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
            }
            logDebug(ss.str());
        }
        
        try {
            // First extract the stream index (first 4 bytes)
            memcpy(&streamIndex, buffer, sizeof(int));
            
            // Format: [streamIndex(4) | sequence(8) | timestamp(8) | position(12) | quaternion(16) | velocity(12) |
            //          current(12) | flags(2) | priority(1)]
            
            // Extract sequence number and timestamp
            memcpy(&sequenceNumber, buffer + 4, sizeof(uint64_t));
            memcpy(&timestamp, buffer + 12, sizeof(uint64_t));
            
            if (g_logConfig.enableDataLogging) {
                logDebug("Extracted stream: " + std::to_string(streamIndex) + 
                        ", sequence: " + std::to_string(sequenceNumber) + 
                        ", timestamp: " + std::to_string(timestamp));
            }
            
            // Extract position data
            if (length >= 20 + 3 * sizeof(float)) {
                memcpy(&posX, buffer + 20, sizeof(float));
                memcpy(&posY, buffer + 24, sizeof(float));
                memcpy(&posZ, buffer + 28, sizeof(float));
                
                // Check for NaN or infinity
                if (!std::isfinite(posX)) posX = 0.0f;
                if (!std::isfinite(posY)) posY = 0.0f;
                if (!std::isfinite(posZ)) posZ = 0.0f;
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Position: [" + std::to_string(posX) + ", " + 
                           std::to_string(posY) + ", " + std::to_string(posZ) + "]");
                }
            } else {
                // Default position if not available
                posX = posY = posZ = 0.0f;
                if (g_logConfig.enableDataLogging) {
                    logDebug("Position data not available, using defaults");
                }
            }
            
            // Set reasonable defaults for quaternion
            quatW = 1.0f;
            quatX = quatY = quatZ = 0.0f;
            
            // Extract quaternion if available
            if (length >= 32 + 4 * sizeof(float)) {
                memcpy(&quatW, buffer + 32, sizeof(float));
                memcpy(&quatX, buffer + 36, sizeof(float));
                memcpy(&quatY, buffer + 40, sizeof(float));
                memcpy(&quatZ, buffer + 44, sizeof(float));
                
                // Safety check for NaN/Inf
                if (!std::isfinite(quatW)) quatW = 1.0f;
                if (!std::isfinite(quatX)) quatX = 0.0f;
                if (!std::isfinite(quatY)) quatY = 0.0f;
                if (!std::isfinite(quatZ)) quatZ = 0.0f;
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Quaternion: [" + std::to_string(quatW) + ", " + 
                           std::to_string(quatX) + ", " + std::to_string(quatY) + 
                           ", " + std::to_string(quatZ) + "]");
                }
            }
            
            // Set reasonable defaults for velocity
            velX = velY = velZ = 0.0f;
            
            // Extract velocity if available
            if (length >= 48 + 3 * sizeof(float)) {
                memcpy(&velX, buffer + 48, sizeof(float));
                memcpy(&velY, buffer + 52, sizeof(float));
                memcpy(&velZ, buffer + 56, sizeof(float));
                
                // Safety check for NaN/Inf
                if (!std::isfinite(velX)) velX = 0.0f;
                if (!std::isfinite(velY)) velY = 0.0f;
                if (!std::isfinite(velZ)) velZ = 0.0f;
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Velocity: [" + std::to_string(velX) + ", " + 
                          std::to_string(velY) + ", " + std::to_string(velZ) + "]");
                }
            }
            
            // Set reasonable defaults for current
            currX = currY = currZ = 0.0f;
            
            // Extract current if available
            if (length >= 60 + 3 * sizeof(float)) {
                memcpy(&currX, buffer + 60, sizeof(float));
                memcpy(&currY, buffer + 64, sizeof(float));
                memcpy(&currZ, buffer + 68, sizeof(float));
                
                // Safety check for NaN/Inf
                if (!std::isfinite(currX)) currX = 0.0f;
                if (!std::isfinite(currY)) currY = 0.0f;
                if (!std::isfinite(currZ)) currZ = 0.0f;
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Current: [" + std::to_string(currX) + ", " + 
                          std::to_string(currY) + ", " + std::to_string(currZ) + "]");
                }
            }
            
            // Default boolean states
            locked = false;
            closeGripper = false;
            
            // Extract boolean states if available
            if (length >= 72 + 2) {
                uint8_t lockState = 0;
                uint8_t gripperState = 0;
                
                memcpy(&lockState, buffer + 72, sizeof(uint8_t));
                memcpy(&gripperState, buffer + 73, sizeof(uint8_t));
                
                locked = (lockState != 0);
                closeGripper = (gripperState != 0);
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Booleans: locked=" + std::string(locked ? "true" : "false") + 
                          ", closeGripper=" + std::string(closeGripper ? "true" : "false"));
                }
            }
            
            // Extract priority if available
            priority = NORMAL;
            if (length >= 74 + 1) {
                uint8_t priorityValue = 0;
                memcpy(&priorityValue, buffer + 74, sizeof(uint8_t));
                priority = static_cast<HapticMessagePriority>(priorityValue);
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Priority: " + std::to_string(priority));
                }
            }
            
            return true;
        } catch (const std::exception& e) {
            logError("Exception in parseFromBuffer: " + std::string(e.what()));
        
            // Initialize all fields to safe defaults
            sequenceNumber = 0;
            timestamp = 0;
            posX = posY = posZ = 0.0f;
            quatW = 1.0f; quatX = quatY = quatZ = 0.0f;
            velX = velY = velZ = 0.0f;
            currX = currY = currZ = 0.0f;
            locked = false;
            closeGripper = false;
            priority = NORMAL;
            streamIndex = 0;
            
            return false;        
        }
    }
};

// Function to analyze client protocol (optional verbose output)
void analyzeClientProtocol(const uint8_t* buffer, uint32_t length) {
    if (!g_logConfig.enableProtocolAnalysis) {
        return; // Skip if protocol analysis is disabled
    }
    
    std::stringstream ss;
    ss << "Client Protocol Analysis:" << std::endl;
    
    // Check for different protocol possibilities
    
    // 1. Check if it could be a string message
    bool couldBeString = true;
    for (uint32_t i = 0; i < length; i++) {
        // Check if all bytes are printable ASCII or null terminator
        if (!(buffer[i] == 0 || (buffer[i] >= 32 && buffer[i] <= 126))) {
            couldBeString = false;
            break;
        }
    }
    
    if (couldBeString) {
        ss << "Possible ASCII string message. As string: \"";
        for (uint32_t i = 0; i < length; i++) {
            if (buffer[i] >= 32 && buffer[i] <= 126) {
                ss << static_cast<char>(buffer[i]);
            } else if (buffer[i] == 0) {
                ss << "\\0";
            }
        }
        ss << "\"" << std::endl;
    }
    
    // 2. Check if it's a pointer to internal memory structure
    // Memory addresses often start with values in certain ranges
    uint64_t possiblePtr = 0;
    if (length >= 8) {
        memcpy(&possiblePtr, buffer, sizeof(uint64_t));
        
        // Check if it looks like a typical 64-bit memory address
        // On most x64 systems, valid heap addresses are often between 0x00007f0000000000 and 0x00007fffffffffff
        if ((possiblePtr & 0xFFFF000000000000) == 0x7F0000000000) {
            ss << "First 8 bytes (0x" << std::hex << possiblePtr << ") match pattern of x64 heap memory address" << std::endl;
        }
        // Process memory often starts with 0x74, 0x55, etc. on some platforms
        else if ((possiblePtr & 0xFF00000000000000) == 0x7400000000000000 ||
                 (possiblePtr & 0xFF00000000000000) == 0x5500000000000000) {
            ss << "First 8 bytes (0x" << std::hex << possiblePtr << ") match pattern of process memory address" << std::endl;
        }
    }
    
    // 3. Try different binary interpretations
    if (length >= 8) {
        // Try as 64-bit integer
        int64_t int64Value;
        memcpy(&int64Value, buffer, sizeof(int64_t));
        ss << "As int64: " << std::dec << int64Value << std::endl;
        
        // Try as 32-bit integers
        if (length >= 8) {
            int32_t int32Value1, int32Value2;
            memcpy(&int32Value1, buffer, sizeof(int32_t));
            memcpy(&int32Value2, buffer + 4, sizeof(int32_t));
            ss << "As two int32: " << int32Value1 << ", " << int32Value2 << std::endl;
        }
        
        // Try as double
        if (length >= 8) {
            double doubleValue;
            memcpy(&doubleValue, buffer, sizeof(double));
            ss << "As double: " << doubleValue << std::endl;
        }
        
        // Try as float
        if (length >= 4) {
            float floatValue;
            memcpy(&floatValue, buffer, sizeof(float));
            ss << "As float: " << floatValue << std::endl;
        }
    }
    
    logDebug(ss.str());
}

// Debug helper function to print bytes in hex (optional)
void dumpHexData(const uint8_t* data, uint32_t length) {
    if (!g_logConfig.enableDataLogging) {
        return; // Skip if data logging is disabled
    }
    
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

// Structure to track ACK batching state for each stream
struct StreamAckState {
    uint64_t messageCount = 0;
    uint64_t lastSequence = 0;
    uint64_t lastTimestamp = 0;
    HapticMessagePriority lastPriority = NORMAL;
    uint64_t* messageCounter = nullptr;
    
    // ACK configuration for surgical applications
    uint64_t ACK_INTERVAL = 1;  // ACK every message for surgical control
    
    // Last ACK time for rate limiting
    std::chrono::steady_clock::time_point lastAckTime;
    
    StreamAckState(uint64_t* counter) : messageCounter(counter) {
        lastAckTime = std::chrono::steady_clock::now();
    }
    
    ~StreamAckState() {
        delete messageCounter;
    }
};

// Inspect data function with verbose output control
void inspectData(const uint8_t* data, size_t length, const char* label) {
    if (!g_logConfig.enableDataLogging) {
        return; // Skip if data logging is disabled
    }
    
    std::stringstream ss;
    ss << "DATA [" << label << "] (" << length << " bytes): ";
    
    // Print first 64 bytes or fewer
    for (size_t i = 0; i < std::min(length, size_t(64)); i++) {
        ss << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(data[i]) << " ";
        if ((i + 1) % 8 == 0) ss << " ";
        if ((i + 1) % 16 == 0) ss << "\n" << std::string(label ? strlen(label) + 8 : 8, ' ');
    }
    
    // Add interpretation of first 28 bytes if we have that many (stream + seq + timestamp + position)
    if (length >= 28) {
        ss << "\n\nInterpretation:";
        
        // Stream index
        int streamIndex = 0;
        memcpy(&streamIndex, data, sizeof(int));
        ss << "\n  Bytes 0-3: " << std::dec << streamIndex << " (stream index)";
        
        // Sequence number
        uint64_t seqNum = 0;
        memcpy(&seqNum, data + 4, sizeof(uint64_t));
        ss << "\n  Bytes 4-11: " << std::dec << seqNum << " (sequence number)";
        
        // Timestamp
        uint64_t timestamp = 0;
        memcpy(&timestamp, data + 12, sizeof(uint64_t));
        ss << "\n  Bytes 12-19: " << timestamp << " (timestamp)";
        
        // Position
        float posX, posY, posZ;
        memcpy(&posX, data + 20, sizeof(float));
        if (length >= 24 + sizeof(float)) {
            memcpy(&posY, data + 24, sizeof(float));
            if (length >= 28 + sizeof(float)) {
                memcpy(&posZ, data + 28, sizeof(float));
                ss << "\n  Position: [" << posX << ", " << posY << ", " << posZ << "]";
            } else {
                ss << "\n  Position: [" << posX << ", " << posY << ", (incomplete)]";
            }
        } else {
            ss << "\n  Position: [" << posX << ", (incomplete)]";
        }
    }
    
    logDebug(ss.str());
}

// Structure to track message statistics and network metrics with improved accuracy
struct NetworkStats {
    std::atomic<uint64_t> messagesReceived{0};
    std::atomic<uint64_t> bytesReceived{0};
    std::chrono::steady_clock::time_point startTime;
    std::chrono::steady_clock::time_point lastMsgTime;
    
    // Latency tracking
    std::vector<double> latencies;
    std::atomic<double> minLatencyMs{std::numeric_limits<double>::max()};
    std::atomic<double> maxLatencyMs{0};
    std::atomic<double> avgLatencyMs{0};
    std::atomic<double> lastLatencyMs{0};
    std::atomic<double> latencySum{0};  // For more efficient average calculation
    std::mutex latencies_mutex;
    
    // Jitter tracking
    std::atomic<double> cumulativeJitter{0};
    std::atomic<double> avgJitterMs{0};
    
    // Sequence number tracking for packet loss detection
    std::atomic<uint64_t> lastSequenceNumber{0};
    std::atomic<uint64_t> packetsLost{0};
    std::atomic<uint64_t> outOfOrderPackets{0};
    std::atomic<uint64_t> duplicatePackets{0};
    std::atomic<bool> receivedFirstPacket{false};
    
    // Priority tracking for surgical applications
    std::atomic<uint64_t> emergencyMessages{0};
    std::atomic<uint64_t> highPriorityMessages{0};
    std::atomic<uint64_t> normalPriorityMessages{0};
    
    // Checksum validation
    std::atomic<uint64_t> checksumErrors{0};

    // Message rate tracking
    std::atomic<uint64_t> lastReportedMessageCount{0};
    std::chrono::steady_clock::time_point lastReportTime;
    
    // Server processing stats
    std::atomic<double> serverProcessingTimeMs{0};
    std::atomic<double> minProcessingTimeMs{std::numeric_limits<double>::max()};
    std::atomic<double> maxProcessingTimeMs{0};
    
    // Thread pool stats
    std::atomic<uint64_t> tasksQueued{0};
    std::atomic<uint64_t> tasksProcessed{0};
    
    // Messages sent back (ACKs)
    std::atomic<uint64_t> messagesSent{0};
    std::atomic<uint64_t> bytesSent{0};
    
    // Per-stream stats
    std::atomic<uint64_t> messagesReceivedPerStream[NUM_STREAMS]{0};
    std::atomic<uint64_t> messagesSentPerStream[NUM_STREAMS]{0};
    
    NetworkStats() {
        startTime = std::chrono::steady_clock::now();
        lastMsgTime = startTime;
        lastReportTime = startTime;
    }

    void update(uint64_t bytes, HapticMessagePriority priority = NORMAL, int streamIndex = 0) {
        messagesReceived++;
        bytesReceived += bytes;
        lastMsgTime = std::chrono::steady_clock::now();
        
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

        // Print statistics periodically based on message count
        if (messagesReceived % g_logConfig.statsInterval == 0) {
            printCompactStatus();
        }
        
        // Print full stats less frequently
        if (messagesReceived % (g_logConfig.statsInterval * 10) == 0) {
            printStats();
        }
    }
    
    void updateSent(uint64_t bytes, int streamIndex = 0) {
        messagesSent++;
        bytesSent += bytes;
        
        // Update stream stats
        if (streamIndex >= 0 && streamIndex < NUM_STREAMS) {
            messagesSentPerStream[streamIndex]++;
        }
    }
    
    void addLatencySample(double latencyMs, uint64_t sequenceNumber) {
        std::lock_guard<std::mutex> lock(latencies_mutex);
        
        // Limit vector size to prevent unbounded growth
        if (latencies.size() >= 1000) {
            latencySum.store(latencySum.load() - latencies[0]);
            latencies.erase(latencies.begin());
        }
        
        // Add latency data
        latencies.push_back(latencyMs);
        latencySum.store(latencySum.load() + latencyMs);
        
        if (latencyMs < minLatencyMs.load()) {
            minLatencyMs.store(latencyMs);
        }
        
        if (latencyMs > maxLatencyMs.load()) {
            maxLatencyMs.store(latencyMs);
        }
        
        // More efficient average calculation
        avgLatencyMs.store(latencySum.load() / latencies.size());
        
        // Calculate jitter (variation in latency)
        if (latencies.size() > 1) {
            double jitter = std::abs(latencyMs - lastLatencyMs.load());
            cumulativeJitter.store(cumulativeJitter.load() + jitter);
            avgJitterMs.store(cumulativeJitter.load() / (latencies.size() - 1));
        }
        lastLatencyMs.store(latencyMs);
        
        // Track package sequence without duplicates
        if (!receivedFirstPacket) {
            receivedFirstPacket = true;
            lastSequenceNumber = sequenceNumber;
        }
        
        // Update last sequence number
        lastSequenceNumber = sequenceNumber;
    }    

    void printCompactStatus() {
        auto now = std::chrono::steady_clock::now();
        double intervalSec = std::chrono::duration<double>(now - lastReportTime).count();
        
        if (intervalSec < 1.0 && messagesReceived != lastReportedMessageCount) {
            // Don't update too frequently
            return;
        }

        // Define color codes
        const std::string CYAN = g_logConfig.colorOutput ? "\033[1;36m" : "";
        const std::string RESET = g_logConfig.colorOutput ? "\033[0m" : "";
        
        // Calculate message rate for this interval
        uint64_t intervalMessages = messagesReceived - lastReportedMessageCount.load();
        double messageRate = intervalSec > 0 ? intervalMessages / intervalSec : 0;
        
        // Calculate overall rate
        double totalTimeSeconds = std::chrono::duration<double>(now - startTime).count();
        double dataRateMbps = (bytesReceived * 8) / (totalTimeSeconds * 1000000);
        
        std::stringstream ss;
        ss << CYAN << "[Status] Msgs: " << messagesReceived << " @ " 
        << std::fixed << std::setprecision(0) << messageRate << "/s | "
        << "Rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps";
        
        // Add latency info if available
        if (!latencies.empty()) {
            ss << " | Latency: " << std::fixed << std::setprecision(3) 
            << avgLatencyMs.load() << " ms";
        }
        
        // Add thread pool info
        ss << " | Active threads: " << g_thread_pool->activeThreadCount()
        << " | Queue: " << g_thread_pool->queueSize();
        
        ss << RESET;
        
        logInfo(ss.str());
        
        // Update tracking variables - fix atomic assignment
        lastReportedMessageCount.store(messagesReceived.load());
        lastReportTime = now;
    }
    
    void incrementChecksumErrors() {
        checksumErrors++;
    }

    void printStats(bool showPacketLoss = true) {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        auto now = std::chrono::steady_clock::now();
        double totalTimeSeconds = std::chrono::duration<double>(now - startTime).count();
        double messageRate = messagesReceived / totalTimeSeconds;
        double dataRateMbps = (bytesReceived * 8) / (totalTimeSeconds * 1000000);

        // Define ANSI color codes
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
        
        // ACK statistics
        std::cout << CYAN << "ACKs sent: " << messagesSent << " (" << bytesSent << " bytes)" << RESET << std::endl;
        
        // Priority statistics
        std::cout << CYAN << "Message priorities: "
                  << RED << "Emergency: " << emergencyMessages << " | "
                  << YELLOW << "High: " << highPriorityMessages << " | "
                  << GREEN << "Normal: " << normalPriorityMessages << RESET << std::endl;
        
        // Per-stream statistics
        std::cout << CYAN << "Per-Stream Statistics:" << RESET << std::endl;
        for (int i = 0; i < NUM_STREAMS; i++) {
            std::cout << CYAN << "  Stream " << i << ": Received " << messagesReceivedPerStream[i] 
                      << ", ACKs sent " << messagesSentPerStream[i] << RESET << std::endl;
        }
        
        if (!latencies.empty()) {
            std::cout << CYAN << "Server processing time (min/avg/max): " << std::fixed << std::setprecision(3)
                      << minLatencyMs << " / " << avgLatencyMs << " / " << maxLatencyMs << " ms" << RESET << std::endl;
            std::cout << CYAN << "Average jitter: " << std::fixed << std::setprecision(3) << avgJitterMs << " ms" << RESET << std::endl;
        }
        
        if (showPacketLoss) {  // Only show packet loss if requested
            // Calculate packet loss percentage
            double lossRate = 0;
            if (messagesReceived + packetsLost > 0) {
                lossRate = (packetsLost * 100.0) / (messagesReceived + packetsLost);
            }
            
            if (packetsLost > 0) {
                std::cout << YELLOW << "Packet loss: " << packetsLost << " packets (" << std::fixed << std::setprecision(1)
                         << lossRate << "%)" << RESET << std::endl;
            }
            
            if (outOfOrderPackets > 0) {
                std::cout << YELLOW << "Out-of-order packets: " << outOfOrderPackets << RESET << std::endl;
            }
        }
        
        if (duplicatePackets > 0) {
            std::cout << YELLOW << "Duplicate packets: " << duplicatePackets << RESET << std::endl;
        }
        if (checksumErrors > 0) {
            std::cout << YELLOW << "Checksum errors: " << checksumErrors << RESET << std::endl;
        }
        
        // Thread pool statistics
        std::cout << CYAN << "Thread pool: " << THREAD_POOL_SIZE << " threads, " 
                  << g_thread_pool->activeThreadCount() << " active, "
                  << g_thread_pool->queueSize() << " queued" << RESET << std::endl;
                  
        std::cout << CYAN << "Running time: " << std::fixed << std::setprecision(1) << totalTimeSeconds << " seconds" << RESET << std::endl;
        std::cout << CYAN << "=============================" << RESET << std::endl;
    }
};

// Global stats tracker
NetworkStats g_stats;

// Global variables for sequence tracking
uint64_t g_expectedSequence = 0;
bool g_receivedFirstMessage = false;
std::unordered_map<uint64_t, std::chrono::steady_clock::time_point> g_messageTimestamps;

// Global ROS2 variables - always defined
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr g_publisher = nullptr;
std::atomic<bool> g_ros_initialized{false};
std::mutex g_ros_mutex;

// Setup DCCP server socket
bool setupDCCPServer(uint16_t port) {
    logInfo("Setting up DCCP server on port " + std::to_string(port));
    
    // Create DCCP socket
    g_server_socket = socket(AF_INET, SOCK_DCCP, IPPROTO_DCCP);
    if (g_server_socket < 0) {
        logError("Failed to create DCCP socket: " + std::string(strerror(errno)));
        return false;
    }
    
    // Set DCCP service code
    int service_code = htonl(DCCP_SERVICE_CODE);
    if (setsockopt(g_server_socket, SOL_DCCP, DCCP_SOCKOPT_SERVICE, &service_code, sizeof(service_code)) < 0) {
        logError("Failed to set DCCP service code: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    // Set socket options for reuse
    int reuseaddr = 1;
    if (setsockopt(g_server_socket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(reuseaddr)) < 0) {
        logError("Failed to set SO_REUSEADDR: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    // Set non-blocking mode
    int flags = fcntl(g_server_socket, F_GETFL, 0);
    if (flags < 0) {
        logError("Failed to get socket flags: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    if (fcntl(g_server_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
        logError("Failed to set socket to non-blocking: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    // Set socket buffer sizes
    int recvbuf = 512 * 1024; // 512KB
    int sendbuf = 512 * 1024; // 512KB
    
    if (setsockopt(g_server_socket, SOL_SOCKET, SO_RCVBUF, &recvbuf, sizeof(recvbuf)) < 0) {
        logWarning("Failed to set receive buffer size: " + std::string(strerror(errno)));
    }
    
    if (setsockopt(g_server_socket, SOL_SOCKET, SO_SNDBUF, &sendbuf, sizeof(sendbuf)) < 0) {
        logWarning("Failed to set send buffer size: " + std::string(strerror(errno)));
    }
    
    // Set CCID (Congestion Control ID) to CCID 2 (TCP-like)
    int ccid = 2; // CCID 2 is TCP-like congestion control
    if (setsockopt(g_server_socket, SOL_DCCP, DCCP_SOCKOPT_CCID, &ccid, sizeof(ccid)) < 0) {
        logWarning("Failed to set CCID: " + std::string(strerror(errno)));
    }
    
    // Bind to address
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
    
    // Listen for connections
    if (listen(g_server_socket, 5) < 0) {
        logError("Failed to listen on socket: " + std::string(strerror(errno)));
        close(g_server_socket);
        g_server_socket = -1;
        return false;
    }
    
    logInfo("DCCP server setup complete on port " + std::to_string(port));
    return true;
}

// Process received haptic data safely with priority handling
void ProcessHapticData(const uint8_t* buffer, uint32_t length, uint64_t globalMsgCount, const struct sockaddr_in& clientAddr) {
    if (!buffer) {
        logWarning("Warning: Null buffer in ProcessHapticData");
        return;
    }
    
    if (length < 16) {
        logWarning("Warning: Buffer too small in ProcessHapticData: " + std::to_string(length) + " bytes");
        return;
    }

    if (g_shutdown_requested.load()) {
        logDebug("Shutdown requested, aborting data processing");
        return;
    }

    try {
        // Record processing start time with high precision
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Add artificial processing delay if configured
        if (g_logConfig.artificialProcessingDelayMs > 0.0) {
            double delayMs = g_logConfig.artificialProcessingDelayMs;
            
            // Add some randomness if enabled
            if (g_logConfig.useRandomProcessingDelay) {
                static std::random_device rd;
                static std::mt19937 gen(rd());
                static std::uniform_real_distribution<> dis(0.0, 1.0);
                delayMs *= dis(gen);  // Random value between 0 and configured max
            }
            
            // Sleep for the desired delay
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<long>(delayMs * 1000)));
        }
        
        // Make a safe copy of the data
        std::vector<uint8_t> dataCopy(buffer, buffer + length);
        
        // Inspect the data for debugging
        inspectData(dataCopy.data(), dataCopy.size(), "ProcessHapticData");
        
        // Parse the message
        HapticMessage msg;
        bool parseSuccess = msg.parseFromBuffer(dataCopy.data(), length);
        
        if (!parseSuccess) {
            logWarning("Failed to parse message data, skipping processing");
            return;
        }
        
        // Additional validation for safety
        if (!std::isfinite(msg.posX) || !std::isfinite(msg.posY) || !std::isfinite(msg.posZ)) {
            logWarning("Message contains non-finite position values, skipping processing");
            return;
        }

        // Calculate server processing latency - use microseconds for greater precision
        double processingLatencyMs = std::chrono::duration<double, std::micro>(
            std::chrono::high_resolution_clock::now() - startTime).count() / 1000.0;
            
        // Log high priority messages immediately for surgical safety
        if (msg.priority == EMERGENCY) {
            logInfo("EMERGENCY MESSAGE #" + std::to_string(msg.sequenceNumber) + 
                   " processing time: " + std::to_string(processingLatencyMs) + " ms");
        }
        else if (msg.priority == HIGH) {
            logInfo("HIGH PRIORITY MESSAGE #" + std::to_string(msg.sequenceNumber) + 
                   " processing time: " + std::to_string(processingLatencyMs) + " ms");
        }
        // For normal messages, log only occasionally
        else if (globalMsgCount % 1000 == 0) {
            logDebug("Message #" + std::to_string(globalMsgCount) + 
                   " processing time: " + std::to_string(processingLatencyMs) + " ms");
        }
            
        // Update message count and stats
        {
            std::lock_guard<std::recursive_mutex> lock(g_stats_mutex);
            
            // Only log first message sequence and not every discontinuity
            if (!g_receivedFirstMessage) {
                g_receivedFirstMessage = true;
                g_expectedSequence = msg.sequenceNumber + 1;
                logInfo("First message received with sequence: " + std::to_string(msg.sequenceNumber));
            } else {
                g_expectedSequence = msg.sequenceNumber + 1;
            }
            
            g_stats.update(length, msg.priority, msg.streamIndex);
            
            // Track message sequence and processing latency
            g_stats.addLatencySample(processingLatencyMs, msg.sequenceNumber);
        }
        
        // Print occasional message info
        if (globalMsgCount % 1000 == 0) {
            logDebug("Message #" + std::to_string(msg.sequenceNumber) + 
                   " (global #" + std::to_string(globalMsgCount) + ")" +
                   " position: " + std::to_string(msg.posX) + ", " + 
                   std::to_string(msg.posY) + ", " + std::to_string(msg.posZ));
        }
        
        // Publish to ROS if initialized
        if (g_ros_initialized && g_publisher && g_node && rclcpp::ok() && !g_shutdown_requested.load()) {
            try {
                auto rosMsg = std::make_shared<omni_msgs::msg::OmniState>();
                
                // Set header
                rosMsg->header.frame_id = "world";
                rosMsg->header.stamp = g_node->now();
                
                // Set message data
                rosMsg->locked = msg.locked;
                rosMsg->close_gripper = msg.closeGripper;
                
                rosMsg->pose.position.x = msg.posX;
                rosMsg->pose.position.y = msg.posY;
                rosMsg->pose.position.z = msg.posZ;
                
                rosMsg->pose.orientation.w = msg.quatW;
                rosMsg->pose.orientation.x = msg.quatX;
                rosMsg->pose.orientation.y = msg.quatY;
                rosMsg->pose.orientation.z = msg.quatZ;
                
                rosMsg->velocity.x = msg.velX;
                rosMsg->velocity.y = msg.velY;
                rosMsg->velocity.z = msg.velZ;
                
                rosMsg->current.x = msg.currX;
                rosMsg->current.y = msg.currY;
                rosMsg->current.z = msg.currZ;
                
                // Publish - prioritize emergency messages
                if (msg.priority == EMERGENCY) {
                    // Use a higher QoS for emergency messages
                    g_publisher->publish(*rosMsg);
                    logInfo("Published EMERGENCY message to ROS: #" + std::to_string(msg.sequenceNumber));
                }
                else if (msg.priority == HIGH) {
                    g_publisher->publish(*rosMsg);
                    if (globalMsgCount % 100 == 0) {
                        logDebug("Published HIGH priority message to ROS: #" + std::to_string(msg.sequenceNumber));
                    }
                }
                else {
                    g_publisher->publish(*rosMsg);
                    if (globalMsgCount % 1000 == 0) {
                        logDebug("Published to ROS: message #" + std::to_string(msg.sequenceNumber));
                    }
                }
            } catch (const std::exception& e) {
                // Log but continue
                if (globalMsgCount % 100 == 0) {
                    logWarning("Error publishing to ROS: " + std::string(e.what()));
                }
            }
        }
        
        // Send acknowledgment back to client
        if (g_server_socket >= 0) {
            // Prepare ACK message
            // Format: [streamIndex(4) | sequenceNumber(8) | timestamp(8)]
            std::vector<uint8_t> ackBuffer(4 + 8 + 8);
            memcpy(ackBuffer.data(), &msg.streamIndex, sizeof(int));
            memcpy(ackBuffer.data() + 4, &msg.sequenceNumber, sizeof(uint64_t));
            memcpy(ackBuffer.data() + 12, &msg.timestamp, sizeof(uint64_t));
            
            // Send ACK
            ssize_t bytesSent = sendto(g_server_socket, ackBuffer.data(), ackBuffer.size(), 0,
                                       (struct sockaddr*)&clientAddr, sizeof(clientAddr));
            
            if (bytesSent < 0) {
                logError("Failed to send ACK: " + std::string(strerror(errno)));
            } else {
                g_stats.updateSent(bytesSent, msg.streamIndex);
                
                // Log for emergency messages
                if (msg.priority == EMERGENCY) {
                    logInfo("Sent ACK for EMERGENCY message #" + std::to_string(msg.sequenceNumber));
                }
                // Log occasionally for other messages
                else if ((msg.priority == HIGH && globalMsgCount % 100 == 0) ||
                         (msg.priority == NORMAL && globalMsgCount % 1000 == 0)) {
                    logDebug("Sent ACK for message #" + std::to_string(msg.sequenceNumber));
                }
            }
        }
    } catch (const std::exception& e) {
        logError("Error processing haptic data: " + std::string(e.what()));
    }
}

// Initialize ROS2 with better error handling
bool InitializeROS() {
    std::lock_guard<std::mutex> lock(g_ros_mutex);
    
    if (g_ros_initialized) return true;
    
    if (g_shutdown_requested) {
        logInfo("Shutdown requested, skipping ROS initialization");
        return false;
    }

    try {
        // Initialize ROS2
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create node with surgical robotics name
        g_node = std::make_shared<rclcpp::Node>("dccp_haptic_server_surgical");
        
        // Create publisher with reliable QoS for surgical applications
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        g_publisher = g_node->create_publisher<omni_msgs::msg::OmniState>(
            "/phantom/remote_state", qos);
            
        // Start a ROS2 spin thread
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
        logInfo("ROS2 initialized with surgical robotic parameters");
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
            // Carefully reset the publisher first
            if (g_publisher) {
                logInfo("Resetting publisher...");
                try {
                    // Create a local copy first for safety
                    auto publisher_copy = g_publisher;
                    g_publisher.reset();
                    publisher_copy.reset();
                    logInfo("Publisher reset complete");
                } catch (...) {
                    logError("Error resetting publisher");
                }
            }
            
            // Then reset the node
            if (g_node) {
                logInfo("Resetting node...");
                try {
                    // Create a local copy first for safety
                    auto node_copy = g_node;
                    g_node.reset();
                    node_copy.reset();
                    logInfo("Node reset complete");
                } catch (...) {
                    logError("Error resetting node");
                }
            }
            
            // Finally shut down ROS
            if (rclcpp::ok()) {
                logInfo("Shutting down ROS2 context...");
                try {
                    rclcpp::shutdown();
                    logInfo("ROS2 context shutdown complete");
                } catch (...) {
                    logError("Error shutting down ROS2");
                }
            }
            
            g_ros_initialized = false;
            logInfo("ROS2 shutdown complete");
        } else {
            logInfo("ROS2 was not initialized, nothing to shut down");
        }
    } catch (const std::exception& e) {
        logError("Error shutting down ROS2: " + std::string(e.what()));
    }
}

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    static int signalCount = 0;
    
    // Handle SIGABRT specially
    if (signal == SIGABRT) {
        std::cerr << "\n*** CRITICAL: Received SIGABRT (signal 6) ***\n";
        std::cerr << "This likely indicates a memory corruption or assertion failure\n";
        std::cerr << "Exiting immediately...\n";
        _exit(128 + signal);
    }
    
    logInfo("Received signal " + std::to_string(signal) + ", initiating graceful shutdown...");
    g_shutdown_requested = true;
    g_shutdown_cv.notify_all();
    
    // Force exit on third signal
    if (++signalCount >= 3) {
        std::cerr << "Forcing exit after multiple termination signals\n";
        _exit(1);  // Force exit
    }
}

// Print available network interfaces
void PrintServerAddresses() {
    logInfo("\n===== NETWORK INTERFACES =====");
    
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    
    if (getifaddrs(&ifaddr) == -1) {
        logError("Failed to get network interfaces");
        return;
    }
    
    // Walk through linked list of interfaces
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;
            
        family = ifa->ifa_addr->sa_family;
        
        // Display IPv4 addresses
        if (family == AF_INET) {
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                    host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0) {
                logError(std::string("getnameinfo() failed: ") + gai_strerror(s));
                continue;
            }
            
            // Skip localhost
            if (strcmp(host, "127.0.0.1") == 0)
                continue;
                
            std::stringstream ss;
            ss << "  " << std::setw(8) << std::left << ifa->ifa_name << " : " << host;
            logInfo(ss.str());
        }
    }
    
    freeifaddrs(ifaddr);
    logInfo("To connect from another machine, use one of these IP addresses.");
    logInfo("For local testing, use 'localhost' or '127.0.0.1'");
    logInfo("=============================");
}

// Function to parse command line arguments for server configuration
void parseCommandLineArgs(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
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
        else if (arg.find("--processing-delay=") == 0) {
            std::string value = arg.substr(18);
            try {
                g_logConfig.artificialProcessingDelayMs = std::stod(value);
                logInfo("Artificial processing delay set to " + value + " ms");
            } catch (...) {
                logWarning("Invalid processing delay: " + value);
            }
        }
        else if (arg == "--random-delay") {
            g_logConfig.useRandomProcessingDelay = true;
            logInfo("Random processing delay enabled");
        }
        else if (arg == "--enable-data-logging") {
            g_logConfig.enableDataLogging = true;
            logInfo("Data logging enabled");
        }
        else if (arg == "--enable-protocol-analysis") {
            g_logConfig.enableProtocolAnalysis = true;
            logInfo("Protocol analysis enabled");
        }
        else if (arg.find("--threads=") == 0) {
            std::string value = arg.substr(10);
            try {
                int threads = std::stoi(value);
                // Will be used when initializing thread pool
                logInfo("Thread pool size set to " + value + " threads");
            } catch (...) {
                logWarning("Invalid thread count: " + value);
            }
        }
    }
}

// Main server thread to accept and process messages
void runServer() {
    logInfo("Starting DCCP server main loop");
    
    // Message counter
    uint64_t globalMsgCount = 0;
    uint8_t buffer[MAX_DCCP_PACKET_SIZE];
    
    // Client structure
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    
    while (!g_shutdown_requested.load()) {
        // Poll for events on the server socket
        struct pollfd pfd;
        pfd.fd = g_server_socket;
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
        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddrLen = sizeof(clientAddr);
        ssize_t bytesRead = recvfrom(g_server_socket, buffer, sizeof(buffer), 0, 
                                     (struct sockaddr*)&clientAddr, &clientAddrLen);
        
        if (bytesRead < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                logError("Error receiving data: " + std::string(strerror(errno)));
            }
            continue;
        }
        
        if (bytesRead == 0) {
            // Empty packet
            continue;
        }
        
        // Update client info
        {
            std::lock_guard<std::mutex> lock(g_clients_mutex);
            uint32_t clientIP = clientAddr.sin_addr.s_addr;
            
            // If this is a new client
            if (g_clients.find(clientIP) == g_clients.end()) {
                char ipStr[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &clientAddr.sin_addr, ipStr, INET_ADDRSTRLEN);
                logInfo("New client connected from: " + std::string(ipStr) + ":" + 
                       std::to_string(ntohs(clientAddr.sin_port)));
            }
            
            // Update client info
            g_clients[clientIP].addr = clientAddr;
            g_clients[clientIP].lastActive = std::chrono::steady_clock::now();
        }
        
        // Increment global message counter
        globalMsgCount++;
        
        // Process the message based on priority
        // Check if we have at least a stream index and sequence number
        if (bytesRead >= 12) {
            int streamIndex;
            memcpy(&streamIndex, buffer, sizeof(int));
            
            // Check if this is an emergency message
            HapticMessagePriority priority = NORMAL;
            if (bytesRead >= 74 + 1) {
                uint8_t priorityValue;
                memcpy(&priorityValue, buffer + 74, sizeof(uint8_t));
                priority = static_cast<HapticMessagePriority>(priorityValue);
            }
            
            if (priority == EMERGENCY) {
                // Process emergency messages immediately
                ProcessHapticData(buffer, bytesRead, globalMsgCount, clientAddr);
            } else {
                // Queue other messages to thread pool
                g_stats.tasksQueued++;
                
                // Copy the buffer and client address for thread safety
                std::vector<uint8_t> dataCopy(buffer, buffer + bytesRead);
                struct sockaddr_in clientAddrCopy = clientAddr;
                
                g_thread_pool->enqueue([dataCopy, bytesRead, globalMsgCount, clientAddrCopy]() {
                    g_stats.tasksProcessed++;
                    ProcessHapticData(dataCopy.data(), bytesRead, globalMsgCount, clientAddrCopy);
                });
            }
        } else {
            logWarning("Received packet too small to process: " + std::to_string(bytesRead) + " bytes");
        }
    }
    
    logInfo("Server main loop exiting");
}

int main(int argc, char* argv[]) {
    g_stats.startTime = std::chrono::steady_clock::now();
    
    // Initialize logging config
    g_logConfig.level = LOG_INFO;
    g_logConfig.statsInterval = 1000;
    g_logConfig.colorOutput = true;
    g_logConfig.showTimestamps = true;
    g_logConfig.enableDataLogging = false;
    g_logConfig.enableProtocolAnalysis = false;
    g_logConfig.artificialProcessingDelayMs = 0.0;
    g_logConfig.useRandomProcessingDelay = false;
    
    logInfo("Starting DCCP Haptic Data Server for Surgical Applications...");
    
    // Check for help argument
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "DCCP Haptic Server - Surgical Robotics Edition - Usage:\n"
                      << "  --verbose, -v          Enable verbose logging\n"
                      << "  --quiet, -q            Show only warnings and errors\n"
                      << "  --no-color             Disable colored output\n"
                      << "  --no-timestamp         Disable timestamps in logs\n"
                      << "  --stats-interval=N     Show statistics every N messages\n"
                      << "  --processing-delay=X   Add artificial processing delay (milliseconds)\n"
                      << "  --random-delay         Randomize the processing delay\n"
                      << "  --enable-data-logging  Enable detailed data logging\n"
                      << "  --enable-protocol-analysis  Enable protocol analysis\n"
                      << "  --threads=N            Set thread pool size (default: CPU cores)\n"
                      << "  --help, -h             Show this help message\n";
            return 0;
        }
    }
    
    // Initialize the thread pool before parsing command line args
    g_thread_pool = std::make_unique<ThreadPool>(THREAD_POOL_SIZE);
    logInfo("Thread pool initialized with " + std::to_string(THREAD_POOL_SIZE) + " worker threads");
    
    // Parse command line arguments
    parseCommandLineArgs(argc, argv);

    // Register signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGABRT, signalHandler);
    
    try {
        // Setup DCCP server
        if (!setupDCCPServer(ServerPort)) {
            logError("Failed to setup DCCP server");
            return 1;
        }
        
        // Show available interfaces
        PrintServerAddresses();
        logInfo("DCCP server started on port " + std::to_string(ServerPort));
        logInfo("Press Enter to stop...");
        
        // Initialize ROS2 for surgical applications
        InitializeROS();
        
        // Start the server thread
        std::thread serverThread(runServer);
        
        // Add a periodic stats reporting thread
        std::thread statsThread([&]() {
            while (!g_shutdown_requested) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                if (!g_shutdown_requested) {
                    g_stats.printCompactStatus();
                }
            }
        });
        
        // Wait for user input or signal
        std::mutex waitMtx;
        std::unique_lock<std::mutex> waitLock(waitMtx);
        g_shutdown_cv.wait(waitLock, [&]() {
            // Check for user input
            if (std::cin.rdbuf()->in_avail() > 0) {
                std::cin.get(); // Consume the character
                return true;
            }
            return g_shutdown_requested.load();
        });
        
        // Signal shutdown
        g_shutdown_requested = true;
        
        // Print final statistics
        g_stats.printStats(false);
        
        // Wait for server thread
        if (serverThread.joinable()) {
            serverThread.join();
        }
        
        // Wait for stats thread
        if (statsThread.joinable()) {
            statsThread.join();
        }
        
        // Clean up
        logInfo("Stopping server...");
        
        if (g_server_socket >= 0) {
            close(g_server_socket);
            g_server_socket = -1;
        }
        
        // Shutdown ROS2
        ShutdownROS();
        
        // Cleanup thread pool
        g_thread_pool.reset();
        
        logInfo("Server stopped");
        return 0;
    } catch (const std::exception& e) {
        logError("Unhandled exception in main: " + std::string(e.what()));
        ShutdownROS();
        g_thread_pool.reset();
        return 1;
    } catch (...) {
        logError("Unknown unhandled exception in main");
        ShutdownROS();
        g_thread_pool.reset();
        return 1;
    }
}