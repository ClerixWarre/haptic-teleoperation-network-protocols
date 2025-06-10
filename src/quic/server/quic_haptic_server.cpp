#include <msquic.h>
#include <msquicp.h>
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
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <random>
#include <condition_variable>
#include <queue>
#include <future>
#include <thread>

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

// Global QUIC API table
const QUIC_API_TABLE* MsQuic = nullptr;

// Server Configuration
const uint16_t ServerPort = 4433;
const char* AlpnStr = "quic-sample";
const char* CertificateFile = "server.cert";
const char* PrivateKeyFile = "server.key";

// Global flags
std::atomic<bool> g_shutdown_requested{false};

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

// Pooled memory approach for emergency ACK buffers - Updated with reference counting
constexpr size_t EMERGENCY_ACK_POOL_SIZE = 64;
struct EmergencyAckBuffer {
    uint8_t buffer[128];  // Buffer for ACK data
    QUIC_BUFFER quicBuffer;
    QUIC_SEND_FLAGS sendFlags;
    std::atomic<int> ref_count{1};  // Reference counting for safety
    std::atomic<bool> in_use{false};
    uint64_t message_id;  // For debugging
    
    // Safe reference management
    void add_ref() {
        ref_count.fetch_add(1, std::memory_order_relaxed);
    }
    
    bool release() {
        int old_count = ref_count.fetch_sub(1, std::memory_order_acq_rel);
        if (old_count == 1) {
            // Last reference, safe to mark as free
            in_use.store(false, std::memory_order_release);
            return true;  // Buffer was freed
        }
        return false;  // Still has references
    }
};

EmergencyAckBuffer g_emergencyAckPool[EMERGENCY_ACK_POOL_SIZE];
std::mutex g_emergencyAckMutex;

// Debug helper function
void debugEmergencyBuffer(const char* action, EmergencyAckBuffer* buf) {
    if (!buf) {
        logError("DEBUG: " + std::string(action) + " - NULL buffer!");
        return;
    }
    
    // Check if buffer is in our pool
    bool isPoolBuffer = false;
    int poolIndex = -1;
    for (int i = 0; i < EMERGENCY_ACK_POOL_SIZE; i++) {
        if (&g_emergencyAckPool[i] == buf) {
            isPoolBuffer = true;
            poolIndex = i;
            break;
        }
    }
    
    logInfo("DEBUG: " + std::string(action) + " - Buffer " + std::to_string((uintptr_t)buf) + 
           " (pool_index=" + std::to_string(poolIndex) + ", is_pool=" + (isPoolBuffer ? "yes" : "no") + 
           ", ref_count=" + std::to_string(buf->ref_count.load()) + 
           ", in_use=" + (buf->in_use.load() ? "yes" : "no") + 
           ", msg_id=" + std::to_string(buf->message_id) + ")");
}

// Function to allocate a buffer from the emergency pool with debug output
EmergencyAckBuffer* allocateEmergencyAckBuffer(uint64_t message_id) {
    std::lock_guard<std::mutex> lock(g_emergencyAckMutex);
    
    logDebug("DEBUG: Looking for emergency buffer for message #" + std::to_string(message_id));
    
    for (int i = 0; i < EMERGENCY_ACK_POOL_SIZE; i++) {
        auto& buf = g_emergencyAckPool[i];
        bool expected = false;
        if (buf.in_use.compare_exchange_weak(expected, true, std::memory_order_acquire)) {
            buf.ref_count.store(1, std::memory_order_relaxed);
            buf.message_id = message_id;
            buf.quicBuffer.Buffer = buf.buffer;
            buf.quicBuffer.Length = 0; // Will be set later
            
            debugEmergencyBuffer("ALLOCATED", &buf);
            return &buf;
        }
    }
    
    logError("DEBUG: Emergency buffer pool exhausted for message #" + std::to_string(message_id));
    return nullptr;
}

// Function to release a buffer back to the pool with debug output
void releaseEmergencyAckBuffer(EmergencyAckBuffer* buf) {
    if (!buf) {
        logError("DEBUG: Attempted to release NULL emergency buffer");
        return;
    }
    
    debugEmergencyBuffer("RELEASE_START", buf);
    
    // Verify this is actually from our pool
    bool is_pool_buffer = false;
    for (int i = 0; i < EMERGENCY_ACK_POOL_SIZE; i++) {
        if (&g_emergencyAckPool[i] == buf) {
            is_pool_buffer = true;
            break;
        }
    }
    
    if (!is_pool_buffer) {
        logError("DEBUG: Attempted to release buffer not from emergency pool! Address: " + 
                std::to_string((uintptr_t)buf));
        return;
    }
    
    if (buf->release()) {
        debugEmergencyBuffer("RELEASE_FREED", buf);
        logInfo("DEBUG: Emergency buffer released and freed for message #" + std::to_string(buf->message_id));
    } else {
        debugEmergencyBuffer("RELEASE_STILLREF", buf);
        logDebug("DEBUG: Emergency buffer released but still has references for message #" + 
                std::to_string(buf->message_id));
    }
}

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
    
    // Default constructor
    HapticMessage() : 
        sequenceNumber(0), timestamp(0),
        posX(0.0f), posY(0.0f), posZ(0.0f),
        quatW(1.0f), quatX(0.0f), quatY(0.0f), quatZ(0.0f),
        velX(0.0f), velY(0.0f), velZ(0.0f),
        currX(0.0f), currY(0.0f), currZ(0.0f),
        locked(false), closeGripper(false),
        priority(NORMAL) {}
        
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
            // Format: [sequence(8) | timestamp(8) | position(12) | quaternion(16) | velocity(12) |
            //          current(12) | flags(2) | priority(1)]
            
            // Extract sequence number and timestamp
            memcpy(&sequenceNumber, buffer, sizeof(uint64_t));
            memcpy(&timestamp, buffer + sizeof(uint64_t), sizeof(uint64_t));
            
            if (g_logConfig.enableDataLogging) {
                logDebug("Extracted sequence: " + std::to_string(sequenceNumber) + 
                        ", timestamp: " + std::to_string(timestamp));
            }
            
            // Extract position data
            if (length >= 16 + 3 * sizeof(float)) {
                memcpy(&posX, buffer + 16, sizeof(float));
                memcpy(&posY, buffer + 20, sizeof(float));
                memcpy(&posZ, buffer + 24, sizeof(float));
                
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
            if (length >= 16 + 7 * sizeof(float)) {
                memcpy(&quatW, buffer + 28, sizeof(float));
                memcpy(&quatX, buffer + 32, sizeof(float));
                memcpy(&quatY, buffer + 36, sizeof(float));
                memcpy(&quatZ, buffer + 40, sizeof(float));
                
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
            if (length >= 44 + 3 * sizeof(float)) {
                memcpy(&velX, buffer + 44, sizeof(float));
                memcpy(&velY, buffer + 48, sizeof(float));
                memcpy(&velZ, buffer + 52, sizeof(float));
                
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
            if (length >= 56 + 3 * sizeof(float)) {
                memcpy(&currX, buffer + 56, sizeof(float));
                memcpy(&currY, buffer + 60, sizeof(float));
                memcpy(&currZ, buffer + 64, sizeof(float));
                
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
            if (length >= 68 + 2) {
                uint8_t lockState = 0;
                uint8_t gripperState = 0;
                
                memcpy(&lockState, buffer + 68, sizeof(uint8_t));
                memcpy(&gripperState, buffer + 69, sizeof(uint8_t));
                
                locked = (lockState != 0);
                closeGripper = (gripperState != 0);
                
                if (g_logConfig.enableDataLogging) {
                    logDebug("Booleans: locked=" + std::string(locked ? "true" : "false") + 
                          ", closeGripper=" + std::string(closeGripper ? "true" : "false"));
                }
            }
            
            // Extract priority if available
            priority = NORMAL;
            if (length >= 70 + 1) {
                uint8_t priorityValue = 0;
                memcpy(&priorityValue, buffer + 70, sizeof(uint8_t));
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
            
            return false;        
        }
    }
};

// Structure to track ACK batching state for each stream
struct StreamAckState {
    uint64_t messageCount = 0;
    uint64_t lastSequence = 0;
    uint64_t lastTimestamp = 0;
    HapticMessagePriority lastPriority = NORMAL;
    uint64_t* messageCounter = nullptr;
    
    // ACK configuration for surgical applications
    uint64_t ACK_INTERVAL = 1;  // ACK every message for surgical control
    
    // List of pending ACKs for batching
    std::vector<std::pair<uint64_t, uint64_t>> pendingAcks;
    
    // Last ACK time for rate limiting
    std::chrono::steady_clock::time_point lastAckTime;
    
    StreamAckState(uint64_t* counter) : messageCounter(counter) {
        lastAckTime = std::chrono::steady_clock::now();
    }
    
    ~StreamAckState() {
        delete messageCounter;
    }
};

// Structure to manage send buffer lifetime on server
struct SendContext {
    std::vector<uint8_t> buffer;  // Directly own the buffer 
    QUIC_BUFFER quicBuffer;
    uint64_t sequenceNumber;
    HapticMessagePriority priority;
    std::chrono::steady_clock::time_point sendTime;
    
    SendContext(const std::vector<uint8_t>& data, uint64_t seqNum, HapticMessagePriority prio = NORMAL) 
        : buffer(data), sequenceNumber(seqNum), priority(prio) {
        sendTime = std::chrono::steady_clock::now();
        quicBuffer.Buffer = buffer.data();
        quicBuffer.Length = static_cast<uint32_t>(buffer.size());
    }
};

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
    
    NetworkStats() {
        startTime = std::chrono::steady_clock::now();
        lastMsgTime = startTime;
        lastReportTime = startTime;
    }

    void update(uint64_t bytes, HapticMessagePriority priority = NORMAL) {
        messagesReceived++;
        bytesReceived += bytes;
        lastMsgTime = std::chrono::steady_clock::now();
        
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
    
    // Fix for the addLatencySample method:
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

    // Fix for the printCompactStatus method:
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
        std::cout << CYAN << "Total bytes: " << bytesReceived << RESET << std::endl;
        std::cout << CYAN << "Messages per second: " << std::fixed << std::setprecision(1) << messageRate << RESET << std::endl;
        std::cout << CYAN << "Data rate: " << std::fixed << std::setprecision(2) << dataRateMbps << " Mbps" << RESET << std::endl;
        
        // Priority statistics
        std::cout << CYAN << "Message priorities: "
                  << RED << "Emergency: " << emergencyMessages << " | "
                  << YELLOW << "High: " << highPriorityMessages << " | "
                  << GREEN << "Normal: " << normalPriorityMessages << RESET << std::endl;
        
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

// Process received haptic data safely with priority handling - Updated to skip emergency ROS2 publishing
void ProcessHapticData(const uint8_t* buffer, uint32_t length, uint64_t globalMsgCount) {
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
            
            g_stats.update(length, msg.priority);
            
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
        
        // Publish to ROS if initialized - BUT NOT for emergency messages (they're handled in stream callback)
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
                
                // SKIP emergency messages - they're handled in the stream callback
                if (msg.priority == EMERGENCY) {
                    // Emergency messages are handled separately in ServerStreamCallback
                    // to avoid race conditions between QUIC and ROS2 threads
                    logDebug("Skipping ROS2 publish for emergency message #" + 
                            std::to_string(msg.sequenceNumber) + " (handled in callback)");
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
        g_node = std::make_shared<rclcpp::Node>("quic_haptic_server_surgical");
        
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

// Forward declarations
QUIC_STATUS ServerStreamCallback(HQUIC Stream, void* Context, QUIC_STREAM_EVENT* Event);
QUIC_STATUS ServerConnectionCallback(HQUIC Connection, void* Context, QUIC_CONNECTION_EVENT* Event);
QUIC_STATUS ServerListenerCallback(HQUIC Listener, void* Context, QUIC_LISTENER_EVENT* Event);

// Complete updated ServerStreamCallback function with debug emergency handling
QUIC_STATUS ServerStreamCallback(HQUIC Stream, void* Context, QUIC_STREAM_EVENT* Event) {
    if (!Event) {
        logError("ERROR: Stream event is null");
        return QUIC_STATUS_INVALID_PARAMETER;
    }

    auto* streamState = static_cast<StreamAckState*>(Context);

    switch (Event->Type) {

    case QUIC_STREAM_EVENT_RECEIVE: {
        const auto& recv = Event->RECEIVE;
    
        if (recv.TotalBufferLength == 0) {
            logDebug("FIN received, no data");
            return QUIC_STATUS_SUCCESS;
        }
    
        if (!recv.Buffers || recv.BufferCount == 0) {
            logDebug("Received empty buffer set");
            return QUIC_STATUS_SUCCESS;
        }
    
        // Copy data - use a shared pointer for safe sharing with thread pool
        auto data = std::make_shared<std::vector<uint8_t>>();
        data->reserve(recv.TotalBufferLength);
        for (uint32_t i = 0; i < recv.BufferCount; ++i) {
            const auto& buf = recv.Buffers[i];
            data->insert(data->end(), buf.Buffer, buf.Buffer + buf.Length);
        }
    
        // Increment the message counter
        uint64_t msgIndex = 0;
        if (streamState && streamState->messageCounter) {
            msgIndex = ++(*streamState->messageCounter);
        }
    
        // Check for priority before queuing
        uint8_t priorityValue = NORMAL;
        if (data->size() >= 71) {  // Check if priority byte exists
            memcpy(&priorityValue, data->data() + 70, sizeof(uint8_t));
        }
        
        // Extract sequence for logging
        uint64_t sequenceNumber = 0;
        if (data->size() >= 8) {
            memcpy(&sequenceNumber, data->data(), sizeof(uint64_t));
        }
    
        // Process ALL messages (including emergency) in thread pool to avoid callback timing issues
        if (priorityValue == EMERGENCY) {
            logInfo("DEBUG: Queueing emergency message #" + std::to_string(sequenceNumber) + " for thread pool processing");
        }
        
        g_stats.tasksQueued++;
        g_thread_pool->enqueue([Stream, data, msgIndex, streamState, priorityValue, sequenceNumber]() {
            g_stats.tasksProcessed++;
            
            HapticMessagePriority priority = static_cast<HapticMessagePriority>(priorityValue);
            
            if (priority == EMERGENCY) {
                logInfo("DEBUG: Starting emergency message processing in thread pool for #" + std::to_string(sequenceNumber));
            }
            
            // Process the message
            ProcessHapticData(data->data(), static_cast<uint32_t>(data->size()), msgIndex);
    
            // Extract timestamp
            uint64_t timestamp = 0;
            if (data->size() >= 16) {
                memcpy(&timestamp, data->data() + sizeof(uint64_t), sizeof(uint64_t));
            }
    
            // Update ACK tracking state
            if (streamState) {
                streamState->messageCount++;
                streamState->lastSequence = sequenceNumber;
                streamState->lastTimestamp = timestamp;
                streamState->lastPriority = priority;
            }
    
            // Handle emergency ACKs
            if (priority == EMERGENCY) {
                logInfo("DEBUG: Processing emergency ACK for message #" + std::to_string(sequenceNumber));
                
                // Get buffer from emergency pool
                EmergencyAckBuffer* ackBuf = allocateEmergencyAckBuffer(sequenceNumber);
                if (!ackBuf) {
                    logError("DEBUG: Emergency ACK pool exhausted! Dropping emergency ACK for #" + 
                            std::to_string(sequenceNumber));
                    MsQuic->StreamReceiveComplete(Stream, data->size());
                    return;
                }
                
                debugEmergencyBuffer("BEFORE_SEND", ackBuf);
                
                // Build the ACK safely
                memcpy(ackBuf->buffer, &sequenceNumber, sizeof(uint64_t));
                memcpy(ackBuf->buffer + sizeof(uint64_t), &timestamp, sizeof(uint64_t));
                ackBuf->quicBuffer.Length = 16;
                ackBuf->sendFlags = QUIC_SEND_FLAG_NONE;
                
                logInfo("DEBUG: About to call MsQuic->StreamSend for emergency message #" + 
                        std::to_string(sequenceNumber));
                
                // Send the ACK
                QUIC_STATUS status = MsQuic->StreamSend(
                    Stream,
                    &ackBuf->quicBuffer,
                    1,
                    ackBuf->sendFlags,
                    ackBuf); // Pass buffer as context
                    
                if (QUIC_FAILED(status)) {
                    logError("DEBUG: StreamSend FAILED for emergency message #" + 
                            std::to_string(sequenceNumber) + " with status 0x" + std::to_string(status));
                    debugEmergencyBuffer("SEND_FAILED", ackBuf);
                    // Release the buffer since MsQuic won't call SEND_COMPLETE
                    releaseEmergencyAckBuffer(ackBuf);
                } else {
                    logInfo("DEBUG: StreamSend SUCCESS for emergency message #" + 
                            std::to_string(sequenceNumber) + " - waiting for SEND_COMPLETE");
                    debugEmergencyBuffer("SEND_SUCCESS", ackBuf);
                }
                
                logInfo("Emergency message #" + std::to_string(sequenceNumber) + " acknowledged (network ACK only)");
            }
            else {
                // Handle normal/high priority ACKs
                bool shouldSendAck = false;
                
                if (priority == HIGH) {
                    // Always ACK high priority messages
                    shouldSendAck = true;
                } else {
                    // For normal messages, use the configured interval
                    shouldSendAck = (streamState && (streamState->messageCount % streamState->ACK_INTERVAL == 0)) || 
                                    (sequenceNumber % 1000 == 0); // Always ACK milestone sequences
                }
    
                // Send ACK if needed
                if (shouldSendAck) {
                    // Create an ACK message with shared ownership
                    std::shared_ptr<std::vector<uint8_t>> responseBuffer = 
                        std::make_shared<std::vector<uint8_t>>(16);
                    memcpy(responseBuffer->data(), &sequenceNumber, sizeof(uint64_t));
                    memcpy(responseBuffer->data() + sizeof(uint64_t), &timestamp, sizeof(uint64_t));
    
                    auto* ctx = new SendContext(*responseBuffer, sequenceNumber, priority);
                    // Use unique_ptr for automatic cleanup if send fails
                    std::unique_ptr<SendContext> safeCtx(ctx);
    
                    QUIC_STATUS status = MsQuic->StreamSend(
                        Stream,
                        &ctx->quicBuffer,
                        1,
                        QUIC_SEND_FLAG_NONE,
                        ctx);
    
                    if (QUIC_FAILED(status)) {
                        logError("Failed to send response: 0x" + std::to_string(status));
                        // ctx will be deleted by the unique_ptr
                    } else {
                        if (priority == HIGH) {
                            if (msgIndex % 100 == 0) {  // Log occasionally
                                logDebug("Sent response for HIGH PRIORITY message #" + std::to_string(msgIndex));
                            }
                        } else if (msgIndex % 1000 == 0) {  // Log less frequently for normal messages
                            logDebug("Sent response for message #" + std::to_string(msgIndex));
                        }
                        // Release ownership - MsQuic will handle cleanup via SEND_COMPLETE event
                        safeCtx.release();
                    }
                }
            }
    
            // Complete the receive for all message types
            MsQuic->StreamReceiveComplete(Stream, data->size());
            
            if (priority == EMERGENCY) {
                logInfo("DEBUG: Emergency message thread pool processing completed for #" + std::to_string(sequenceNumber));
            }
        });
    
        return QUIC_STATUS_PENDING;
    }

    case QUIC_STREAM_EVENT_SEND_COMPLETE: {
        logDebug("DEBUG: SEND_COMPLETE event received");
        
        if (Event->SEND_COMPLETE.ClientContext) {
            void* ctxPtr = Event->SEND_COMPLETE.ClientContext;
            
            logDebug("DEBUG: SEND_COMPLETE with context " + std::to_string((uintptr_t)ctxPtr));
            
            // Check if this is an emergency buffer
            bool isEmergencyBuffer = false;
            for (int i = 0; i < EMERGENCY_ACK_POOL_SIZE; i++) {
                if (&g_emergencyAckPool[i] == static_cast<EmergencyAckBuffer*>(ctxPtr)) {
                    isEmergencyBuffer = true;
                    break;
                }
            }
            
            if (isEmergencyBuffer) {
                EmergencyAckBuffer* emergencyBuf = static_cast<EmergencyAckBuffer*>(ctxPtr);
                logInfo("DEBUG: SEND_COMPLETE for emergency buffer, message #" + 
                       std::to_string(emergencyBuf->message_id));
                debugEmergencyBuffer("SEND_COMPLETE", emergencyBuf);
                
                logInfo("DEBUG: About to call releaseEmergencyAckBuffer");
                releaseEmergencyAckBuffer(emergencyBuf);
                logInfo("DEBUG: releaseEmergencyAckBuffer completed");
            } else {
                logDebug("DEBUG: SEND_COMPLETE for normal buffer");
                // Normal message ACK completed
                SendContext* ctx = static_cast<SendContext*>(ctxPtr);
                delete ctx;
            }
        } else {
            logDebug("DEBUG: SEND_COMPLETE with NULL context");
        }
        
        logDebug("DEBUG: SEND_COMPLETE event handling completed");
        return QUIC_STATUS_SUCCESS;
    }

    case QUIC_STREAM_EVENT_PEER_SEND_SHUTDOWN:
        logDebug("Client finished sending data (peer shutdown)");
        return QUIC_STATUS_SUCCESS;

    case QUIC_STREAM_EVENT_PEER_SEND_ABORTED:
        logDebug("Peer aborted send");
        return QUIC_STATUS_SUCCESS;

    case QUIC_STREAM_EVENT_PEER_RECEIVE_ABORTED:
        logDebug("Peer aborted receive");
        return QUIC_STATUS_SUCCESS;

    case QUIC_STREAM_EVENT_SHUTDOWN_COMPLETE:
        logDebug("Stream shutdown complete");
        if (Context) {
            try {
                delete static_cast<StreamAckState*>(Context);
            } catch (...) {
                logWarning("Warning: Exception during stream context cleanup.");
            }
        }
        MsQuic->StreamClose(Stream);
        return QUIC_STATUS_SUCCESS;

    default:
        logDebug("Unhandled stream event: " + std::to_string(Event->Type));
        return QUIC_STATUS_SUCCESS;
    }
}

// Listener callback implementation
QUIC_STATUS ServerListenerCallback(HQUIC Listener, void* Context, QUIC_LISTENER_EVENT* Event) {
    if (!Event) {
        logError("ERROR: Listener event pointer is null");
        return QUIC_STATUS_INVALID_PARAMETER;
    }
    
    try {
        switch (Event->Type) {
            case QUIC_LISTENER_EVENT_NEW_CONNECTION: {
                logInfo("New connection received");
                
                if (!Event->NEW_CONNECTION.Connection) {
                    logError("ERROR: Connection pointer is null");
                    return QUIC_STATUS_INVALID_PARAMETER;
                }
                
                // Set optimized connection settings for surgical applications
                QUIC_SETTINGS Settings = {0};
                Settings.IsSet.PeerBidiStreamCount = 1;
                Settings.PeerBidiStreamCount = 20;  // Allow up to 20 bidirectional streams
                Settings.IsSet.PeerUnidiStreamCount = 1;
                Settings.PeerUnidiStreamCount = 20; // Allow up to 20 unidirectional streams
                
                // Critical for surgical haptic data
                Settings.IsSet.MaxAckDelayMs = 1;
                Settings.MaxAckDelayMs = 1;  // 1ms for ultra-low latency
                
                // Disable buffering for surgical real-time applications
                Settings.IsSet.SendBufferingEnabled = 1;
                Settings.SendBufferingEnabled = 0;
                
                // Apply settings to connection
                QUIC_STATUS paramStatus = MsQuic->SetParam(
                    Event->NEW_CONNECTION.Connection,
                    QUIC_PARAM_CONN_SETTINGS,
                    sizeof(Settings),
                    &Settings);
                    
                if (QUIC_FAILED(paramStatus)) {
                    logError("Failed to set connection parameters");
                }
                
                // Create message counter for this connection
                uint64_t* messageCounter = new uint64_t(0);
                
                // Set connection callback
                MsQuic->SetCallbackHandler(
                    Event->NEW_CONNECTION.Connection,
                    (void*)ServerConnectionCallback,
                    messageCounter);
                
                // Set configuration
                QUIC_STATUS status = MsQuic->ConnectionSetConfiguration(
                    Event->NEW_CONNECTION.Connection,
                    (HQUIC)Context);
                    
                if (QUIC_FAILED(status)) {
                    logError("ConnectionSetConfiguration failed");
                    delete messageCounter;
                    return QUIC_STATUS_CONNECTION_REFUSED;
                }
                
                logInfo("Connection accepted");
                return QUIC_STATUS_SUCCESS;
            }
                
            default:
                logDebug("Unhandled listener event type: " + std::to_string(Event->Type));
                return QUIC_STATUS_SUCCESS;
        }
    } catch (const std::exception& e) {
        logError("Exception in listener callback: " + std::string(e.what()));
        return QUIC_STATUS_INTERNAL_ERROR;
    } catch (...) {
        logError("Unknown exception in listener callback");
        return QUIC_STATUS_INTERNAL_ERROR;
    }
    
    return QUIC_STATUS_SUCCESS;
}

// Generate self-signed certificate
bool CreateSelfSignedCert() {
    logInfo("Generating self-signed certificate...");
    
    // Remove old certificates if they exist
    if (std::ifstream(CertificateFile)) {
        remove(CertificateFile);
    }
    if (std::ifstream(PrivateKeyFile)) {
        remove(PrivateKeyFile);
    }
    
    // Use simple RSA key and basic certificate configuration
    std::string genKeyCmd = "openssl genrsa -out " + std::string(PrivateKeyFile) + " 2048";
    std::string genCertCmd = "openssl req -new -x509 -sha256 -key " + std::string(PrivateKeyFile) + 
                            " -out " + std::string(CertificateFile) + 
                            " -days 365 -subj '/CN=localhost'";
    
    // Generate key
    int keyResult = system(genKeyCmd.c_str());
    if (keyResult != 0) {
        logError("Failed to generate private key, error code: " + std::to_string(keyResult));
        return false;
    }
    
    // Generate certificate
    int certResult = system(genCertCmd.c_str());
    if (certResult != 0) {
        logError("Failed to generate certificate, error code: " + std::to_string(certResult));
        return false;
    }
    
    // Verify files exist
    std::ifstream keyFile(PrivateKeyFile);
    std::ifstream certFile(CertificateFile);
    if (!keyFile.good() || !certFile.good()) {
        logError("Certificate files not created properly");
        return false;
    }
    
    logInfo("Certificate generated successfully");
    return true;
}

// Get credential configuration
QUIC_CREDENTIAL_CONFIG* GetCredentialConfig() {
    static QUIC_CREDENTIAL_CONFIG CredConfig;
    static QUIC_CERTIFICATE_FILE CertFile;
    
    // Initialize structures
    memset(&CredConfig, 0, sizeof(CredConfig));
    memset(&CertFile, 0, sizeof(CertFile));
    
    // Set certificate file paths
    CertFile.CertificateFile = CertificateFile;
    CertFile.PrivateKeyFile = PrivateKeyFile;
    
    // Set basic configuration
    CredConfig.Type = QUIC_CREDENTIAL_TYPE_CERTIFICATE_FILE;
    CredConfig.Flags = QUIC_CREDENTIAL_FLAG_NONE;
    CredConfig.CertificateFile = &CertFile;
    
    return &CredConfig;
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
    }
}

// Connection callback implementation
QUIC_STATUS ServerConnectionCallback(HQUIC Connection, void* Context, QUIC_CONNECTION_EVENT* Event) {
    switch (Event->Type) {

    case QUIC_CONNECTION_EVENT_CONNECTED:
        logInfo("Client connected");
        return QUIC_STATUS_SUCCESS;

    case QUIC_CONNECTION_EVENT_SHUTDOWN_INITIATED_BY_TRANSPORT:
        logInfo("Connection shutdown initiated by transport");
        return QUIC_STATUS_SUCCESS;

    case QUIC_CONNECTION_EVENT_SHUTDOWN_INITIATED_BY_PEER:
        logInfo("Connection shutdown initiated by peer");
        return QUIC_STATUS_SUCCESS;

    case QUIC_CONNECTION_EVENT_SHUTDOWN_COMPLETE: {
        logInfo("Connection shutdown complete");

        // Free the per-connection message counter
        if (Context) {
            try {
                delete static_cast<uint64_t*>(Context);
            } catch (...) {
                logWarning("Warning: Exception during context cleanup.");
            }
        }

        // Clean up connection handle
        MsQuic->ConnectionClose(Connection);
        return QUIC_STATUS_SUCCESS;
    }

    case QUIC_CONNECTION_EVENT_PEER_STREAM_STARTED: {
        logDebug("New stream started by client");
    
        if (!Event->PEER_STREAM_STARTED.Stream) {
            logError("ERROR: Stream pointer is null in PEER_STREAM_STARTED event");
            return QUIC_STATUS_INVALID_PARAMETER;
        }
    
        // Instead of sharing the message counter, allocate one per stream
        auto* streamCounter = new uint64_t(0);
        
        // Create the stream ACK state with configured values
        auto* streamState = new StreamAckState(streamCounter);
        streamState->ACK_INTERVAL = 1;  // ACK every message for surgical applications
    
        MsQuic->SetCallbackHandler(
            Event->PEER_STREAM_STARTED.Stream,
            (void*)ServerStreamCallback,
            streamState);
    
        logDebug("Stream callback set successfully");
        return QUIC_STATUS_SUCCESS;
    }

    default:
        logDebug("Unhandled connection event type: " + std::to_string(Event->Type));
        return QUIC_STATUS_SUCCESS;
    }
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
    
    logInfo("Starting QUIC Haptic Data Server for Surgical Applications...");
    
    // Check for help argument
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "QUIC Haptic Server - Surgical Robotics Edition - Usage:\n"
                      << "  --verbose, -v          Enable verbose logging\n"
                      << "  --quiet, -q            Show only warnings and errors\n"
                      << "  --no-color             Disable colored output\n"
                      << "  --no-timestamp         Disable timestamps in logs\n"
                      << "  --stats-interval=N     Show statistics every N messages\n"
                      << "  --processing-delay=X   Add artificial processing delay (milliseconds)\n"
                      << "  --random-delay         Randomize the processing delay\n"
                      << "  --enable-data-logging  Enable detailed data logging\n"
                      << "  --enable-protocol-analysis  Enable protocol analysis\n"
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
        // Generate certificate
        if (!CreateSelfSignedCert()) {
            return 1;
        }
        
        // Open MSQUIC API
        QUIC_STATUS status;
        if (QUIC_FAILED(status = MsQuicOpen2(&MsQuic))) {
            logError("MsQuicOpen2 failed: 0x" + std::to_string(status));
            return 1;
        }
        
        // Register with QUIC
        HQUIC Registration = nullptr;
        const QUIC_REGISTRATION_CONFIG RegConfig = { "quic-haptic-server", QUIC_EXECUTION_PROFILE_LOW_LATENCY };
        
        if (QUIC_FAILED(status = MsQuic->RegistrationOpen(&RegConfig, &Registration))) {
            logError("RegistrationOpen failed: 0x" + std::to_string(status));
            MsQuicClose(MsQuic);
            return 1;
        }
        
        // Create ALPN buffer
        QUIC_BUFFER AlpnBuffer = { (uint32_t)strlen(AlpnStr), (uint8_t*)AlpnStr };
        
        // Create server configuration with settings optimized for surgical applications
        QUIC_SETTINGS Settings = {0};
        
        // Critical for surgical haptic data
        Settings.IsSet.MaxAckDelayMs = 1;
        Settings.MaxAckDelayMs = 1;  // 1ms for ultra-low latency

        Settings.IsSet.KeepAliveIntervalMs = 1;
        Settings.KeepAliveIntervalMs = 5000;  // 5 seconds for medical applications

        Settings.IsSet.IdleTimeoutMs = 1;
        Settings.IdleTimeoutMs = 600000;  // 10 minutes for surgical procedures
        
        // Disable buffering for more predictable behavior
        Settings.IsSet.SendBufferingEnabled = 1;
        Settings.SendBufferingEnabled = 0;
        
        logInfo("Using optimized server configuration for surgical applications");
        
        // Create configuration
        HQUIC Configuration = nullptr;
        if (QUIC_FAILED(status = MsQuic->ConfigurationOpen(
                Registration, &AlpnBuffer, 1, &Settings, sizeof(Settings), nullptr, &Configuration))) {
            logError("ConfigurationOpen failed: 0x" + std::to_string(status));
            MsQuic->RegistrationClose(Registration);
            MsQuicClose(MsQuic);
            return 1;
        }
        
        // Load certificate
        QUIC_CREDENTIAL_CONFIG* CredConfig = GetCredentialConfig();
        if (QUIC_FAILED(status = MsQuic->ConfigurationLoadCredential(Configuration, CredConfig))) {
            logError("ConfigurationLoadCredential failed: 0x" + std::to_string(status));
            MsQuic->ConfigurationClose(Configuration);
            MsQuic->RegistrationClose(Registration);
            MsQuicClose(MsQuic);
            return 1;
        }
        
        // Create listener
        HQUIC Listener = nullptr;
        if (QUIC_FAILED(status = MsQuic->ListenerOpen(
                Registration, ServerListenerCallback, Configuration, &Listener))) {
            logError("ListenerOpen failed: 0x" + std::to_string(status));
            MsQuic->ConfigurationClose(Configuration);
            MsQuic->RegistrationClose(Registration);
            MsQuicClose(MsQuic);
            return 1;
        }
        
        // Set up listen address - using IPv4 only
        QUIC_ADDR Address{};
        QuicAddrSetFamily(&Address, QUIC_ADDRESS_FAMILY_INET);
        QuicAddrSetPort(&Address, ServerPort);
        
        logInfo("Server binding to IPv4 address, port " + std::to_string(ServerPort));
        
        // Start listening
        if (QUIC_FAILED(status = MsQuic->ListenerStart(Listener, &AlpnBuffer, 1, &Address))) {
            logError("ListenerStart failed: 0x" + std::to_string(status));
            MsQuic->ListenerClose(Listener);
            MsQuic->ConfigurationClose(Configuration);
            MsQuic->RegistrationClose(Registration);
            MsQuicClose(MsQuic);
            return 1;
        }

        logInfo("QUIC server started on port " + std::to_string(ServerPort));
        PrintServerAddresses();
        logInfo("Press Enter to stop...");
        
        // Initialize ROS2 for surgical applications
        InitializeROS();
        
        // Add a periodic stats reporting thread
        std::thread([&]() {
            while (!g_shutdown_requested) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                if (!g_shutdown_requested) {
                    g_stats.printCompactStatus();
                }
            }
        }).detach();
        
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
        
        // Clean up
        logInfo("Stopping server...");
        
        if (Listener) MsQuic->ListenerClose(Listener);
        if (Configuration) MsQuic->ConfigurationClose(Configuration);
        if (Registration) MsQuic->RegistrationClose(Registration);
        if (MsQuic) MsQuicClose(MsQuic);
        
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
