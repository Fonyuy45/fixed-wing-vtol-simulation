/**
 * FIXAR 007 Flight Controller Client (Windows/Ubuntu)
 * High-performance C++ implementation for 400+ Hz communication
 * WSL-ready architecture
 */

#ifndef WINDOWS_FLIGHT_CONTROLLER_HPP_
#define WINDOWS_FLIGHT_CONTROLLER_HPP_

#include <string>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <memory>
#include <chrono>
#include <functional>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef SOCKET socket_t;
    #define CLOSE_SOCKET closesocket
    #define SOCKET_INVALID INVALID_SOCKET
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int socket_t;
    // FIXED: Added '::' to force using the global system close() function
    // instead of the class member close() function.
    #define CLOSE_SOCKET ::close 
    #define SOCKET_INVALID -1
#endif

namespace fixar_client {

/**
 * Sensor data structure
 */
struct SensorData {
    std::array<double, 3> rates;           // rad/s (p, q, r)
    std::array<double, 3> accelerations;   // m/s² (ax, ay, az)
    std::array<double, 3> attitude;        // rad (roll, pitch, yaw)
    std::array<double, 3> position;        // [lat°, lon°, alt_m]
    std::array<double, 3> velocity;        // m/s (vN, vE, vD)
    double azimuth;                        // rad (magnetic heading)
    double timestamp;                      // seconds
    
    SensorData() 
        : rates{0.0, 0.0, 0.0}
        , accelerations{0.0, 0.0, 0.0}
        , attitude{0.0, 0.0, 0.0}
        , position{0.0, 0.0, 0.0}
        , velocity{0.0, 0.0, 0.0}
        , azimuth(0.0)
        , timestamp(0.0) {}
};

/**
 * Windows Flight Controller Client
 */
class WindowsFlightController {
public:
    WindowsFlightController(const std::string& host = "localhost", int port = 5555);
    ~WindowsFlightController();
    
    // Connection management
    bool connect();
    void close();
    bool isConnected() const { return connected_; }
    
    // Motor control
    bool sendMotors(int m1, int m2, int m3, int m4);
    std::array<int, 4> getMotors() const;
    
    // Sensor data access
    bool getSensorData(SensorData& data) const;
    bool hasSensorData() const;
    
    // Test scenarios
    void testHoverAttempt();

private:
    // Network operations
    void receiveData();
    void printStats();
    
    // Utility
    void clearScreen();
    void printBanner();
    std::string formatMotorPacket(int m1, int m2, int m3, int m4);
    
    // Configuration
    std::string host_;
    int port_;
    
    // Socket
    socket_t socket_;
    std::atomic<bool> connected_;
    
    // Sensor data (thread-safe)
    SensorData sensor_data_;
    mutable std::mutex data_mutex_;
    std::atomic<bool> has_data_;
    
    // Motor commands
    std::array<int, 4> motors_;
    mutable std::mutex motor_mutex_;
    
    // Performance monitoring
    std::atomic<uint64_t> rx_counter_;
    std::atomic<uint64_t> tx_counter_;
    std::chrono::steady_clock::time_point last_stats_time_;
    std::deque<double> rx_hz_history_;
    std::deque<double> tx_hz_history_;
    mutable std::mutex stats_mutex_;
    
    // Threads
    std::unique_ptr<std::thread> rx_thread_;
    std::unique_ptr<std::thread> stats_thread_;
    
    // Shutdown flag
    std::atomic<bool> shutdown_requested_;
    
#ifdef _WIN32
    bool wsa_initialized_;
#endif
};

}  // namespace fixar_client

#endif  // WINDOWS_FLIGHT_CONTROLLER_HPP_
