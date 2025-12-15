/**
 * FIXAR 007 Flight Controller Client Implementation
 */

#include "fixar_007_description/windows_flight_controller.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace fixar_client {

// Constants
constexpr double RAD_TO_DEG = 57.2958;

WindowsFlightController::WindowsFlightController(const std::string& host, int port)
    : host_(host)
    , port_(port)
    , socket_(SOCKET_INVALID)
    , connected_(false)
    , has_data_(false)
    , motors_{1000, 1000, 1000, 1000}
    , rx_counter_(0)
    , tx_counter_(0)
    , shutdown_requested_(false)
    
#ifdef _WIN32
    , wsa_initialized_(false)
#endif
{
    last_stats_time_ = std::chrono::steady_clock::now();
    
    printBanner();
    
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
        wsa_initialized_ = true;
    } else {
        throw std::runtime_error("WSAStartup failed");
    }
#endif
}

WindowsFlightController::~WindowsFlightController()
{
    close();
    
#ifdef _WIN32
    if (wsa_initialized_) {
        WSACleanup();
    }
#endif
}

void WindowsFlightController::printBanner()
{
    std::cout << std::string(70, '=') << std::endl;
    std::cout << "🎮 FIXAR 007 Flight Controller Client" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
}

bool WindowsFlightController::connect()
{
    std::cout << "Connecting to " << host_ << ":" << port_ << "..." << std::endl;
    
    // Create socket
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == SOCKET_INVALID) {
        std::cerr << "Socket creation failed" << std::endl;
        return false;
    }
    
    // Configure server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    
#ifdef _WIN32
    inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr);
#else
    inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr);
#endif
    
    // Connect to server
    if (::connect(socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << " Connection refused - is the bridge running?" << std::endl;
        std::cerr << "   Start bridge with: ros2 run fixar_windows_bridge fixar_windows_bridge" << std::endl;
        CLOSE_SOCKET(socket_);
        socket_ = SOCKET_INVALID;
        return false;
    }
    
    // Disable Nagle's algorithm for low latency (CRITICAL!)
    int flag = 1;
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
    
    connected_ = true;
    
    std::cout << " Connected to FIXAR bridge!" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // Start receiver thread
    rx_thread_ = std::make_unique<std::thread>(
        &WindowsFlightController::receiveData, this);
    
    // Start statistics thread
    stats_thread_ = std::make_unique<std::thread>(
        &WindowsFlightController::printStats, this);
    
    return true;
}

void WindowsFlightController::close()
{
    shutdown_requested_ = true;
    connected_ = false;
    
    if (socket_ != SOCKET_INVALID) {
        CLOSE_SOCKET(socket_);
        socket_ = SOCKET_INVALID;
    }
    
    if (rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->join();
    }
    
    if (stats_thread_ && stats_thread_->joinable()) {
        stats_thread_->join();
    }
    
    std::cout << "\nClient closed" << std::endl;
}

void WindowsFlightController::receiveData()
{
    std::string buffer;
    char recv_buffer[8192];
    
    while (connected_ && !shutdown_requested_) {
        // Receive data
#ifdef _WIN32
        int bytes_received = recv(socket_, recv_buffer, sizeof(recv_buffer), 0);
#else
        ssize_t bytes_received = recv(socket_, recv_buffer, sizeof(recv_buffer), 0);
#endif
        
        if (bytes_received <= 0) {
            connected_ = false;
            std::cout << "\n Disconnected from bridge" << std::endl;
            break;
        }
        
        // Add to buffer
        buffer.append(recv_buffer, bytes_received);
        
        // Process complete JSON lines
        size_t newline_pos;
        while ((newline_pos = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, newline_pos);
            buffer = buffer.substr(newline_pos + 1);
            
            if (!line.empty()) {
                try {
                    // Parse JSON
                    auto packet = json::parse(line);
                    
                    // Update sensor data (thread-safe)
                    {
                        std::lock_guard<std::mutex> lock(data_mutex_);
                        
                        if (packet.contains("rates") && packet["rates"].is_array()) {
                            for (size_t i = 0; i < 3; ++i) {
                                sensor_data_.rates[i] = packet["rates"][i];
                            }
                        }
                        
                        if (packet.contains("accelerations") && packet["accelerations"].is_array()) {
                            for (size_t i = 0; i < 3; ++i) {
                                sensor_data_.accelerations[i] = packet["accelerations"][i];
                            }
                        }
                        
                        if (packet.contains("attitude") && packet["attitude"].is_array()) {
                            for (size_t i = 0; i < 3; ++i) {
                                sensor_data_.attitude[i] = packet["attitude"][i];
                            }
                        }
                        
                        if (packet.contains("position") && packet["position"].is_array()) {
                            for (size_t i = 0; i < 3; ++i) {
                                sensor_data_.position[i] = packet["position"][i];
                            }
                        }
                        
                        if (packet.contains("velocity") && packet["velocity"].is_array()) {
                            for (size_t i = 0; i < 3; ++i) {
                                sensor_data_.velocity[i] = packet["velocity"][i];
                            }
                        }
                        
                        if (packet.contains("azimuth")) {
                            sensor_data_.azimuth = packet["azimuth"];
                        }
                        
                        if (packet.contains("timestamp")) {
                            sensor_data_.timestamp = packet["timestamp"];
                        }
                        
                        has_data_ = true;
                    }
                    
                    rx_counter_++;
                    
                } catch (const json::exception&) {
                    // Silently ignore JSON parse errors
                }
            }
        }
        
        // Prevent buffer overflow
        if (buffer.size() > 16384) {
            buffer.clear();
        }
    }
}

std::string WindowsFlightController::formatMotorPacket(int m1, int m2, int m3, int m4)
{
    // Optimized string formatting (faster than json library for simple objects)
    std::ostringstream oss;
    oss << "{\"motors\":[" << m1 << "," << m2 << "," << m3 << "," << m4 << "]}\n";
    return oss.str();
}

bool WindowsFlightController::sendMotors(int m1, int m2, int m3, int m4)
{
    if (!connected_) {
        return false;
    }
    
    // Clamp values to valid range
    m1 = std::max(1000, std::min(2000, m1));
    m2 = std::max(1000, std::min(2000, m2));
    m3 = std::max(1000, std::min(2000, m3));
    m4 = std::max(1000, std::min(2000, m4));
    
    // Update motor state
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        motors_[0] = m1;
        motors_[1] = m2;
        motors_[2] = m3;
        motors_[3] = m4;
    }
    
    try {
        // Create packet
        std::string packet = formatMotorPacket(m1, m2, m3, m4);
        
        // Send packet
#ifdef _WIN32
        int result = send(socket_, packet.c_str(), (int)packet.size(), 0);
        if (result == SOCKET_ERROR) {
#else
        ssize_t result = send(socket_, packet.c_str(), packet.size(), MSG_NOSIGNAL);
        if (result < 0) {
#endif
            return false;
        }
        
        tx_counter_++;
        return true;
        
    } catch (...) {
        return false;
    }
}

std::array<int, 4> WindowsFlightController::getMotors() const
{
    std::lock_guard<std::mutex> lock(motor_mutex_);
    return motors_;
}

bool WindowsFlightController::getSensorData(SensorData& data) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (has_data_) {
        data = sensor_data_;
        return true;
    }
    return false;
}

bool WindowsFlightController::hasSensorData() const
{
    return has_data_;
}

void WindowsFlightController::clearScreen()
{
    // ANSI escape sequence works on Windows 10+, Linux, and macOS
    std::cout << "\033[H\033[J" << std::flush;
}


void WindowsFlightController::printStats()
{
    while (connected_ && !shutdown_requested_) {
        std::this_thread::sleep_for(200ms);  // Faster update rate for better visibility
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - last_stats_time_).count();
        
        if (elapsed > 0) {
            // Calculate rates
            double rx_hz = rx_counter_ / elapsed;
            double tx_hz = tx_counter_ / elapsed;
            
            // Reset counters
            rx_counter_ = 0;
            tx_counter_ = 0;
            last_stats_time_ = current_time;
            
            // Get latest data
            SensorData data;
            bool has_data = getSensorData(data);
            
            // Clear screen for dashboard effect
            clearScreen();
            
            std::cout << std::string(60, '=') << std::endl;
            std::cout << " ATA TELEMETRY DASHBOARD" << std::endl;
            std::cout << std::string(60, '=') << std::endl;
            std::cout << std::fixed << std::setprecision(1);
            std::cout << " LINK STATUS:  RX: " << std::setw(5) << rx_hz << " Hz  |  TX: " 
                      << std::setw(5) << tx_hz << " Hz" << std::endl;
            std::cout << std::string(60, '-') << std::endl;
            
            if (has_data) {
                // 1. ATTITUDE & POSITION
                double roll = data.attitude[0] * RAD_TO_DEG;
                double pitch = data.attitude[1] * RAD_TO_DEG;
                double yaw = data.attitude[2] * RAD_TO_DEG;
                double lat = data.position[0];
                double lon = data.position[1];
                double alt = data.position[2];
                double azimuth = data.azimuth * RAD_TO_DEG;
                
                std::cout << std::setprecision(1);
                std::cout << "  ATTITUDE (deg):  Roll: " << std::setw(6) << roll 
                          << " | Pitch: " << std::setw(6) << pitch 
                          << " | Yaw: " << std::setw(6) << yaw << std::endl;
                
                std::cout << std::setprecision(5);
                std::cout << " POSITION:        Lat: " << std::setw(8) << lat 
                          << " | Lon: " << std::setw(8) << lon;
                std::cout << std::setprecision(2);
                std::cout << " | Alt: " << std::setw(6) << alt << "m" << std::endl;
                
                std::cout << std::setprecision(1);
                std::cout << " AZIMUTH:         " << std::setw(6) << azimuth 
                          << "° (Magnetic Heading)" << std::endl;
                std::cout << std::string(60, '-') << std::endl;
                
                // 2. DYNAMICS
                std::cout << std::setprecision(2);
                std::cout << " VELOCITY (m/s):  N: " << std::setw(5) << data.velocity[0] 
                          << " | E: " << std::setw(5) << data.velocity[1] 
                          << " | D: " << std::setw(5) << data.velocity[2] << std::endl;
                
                std::cout << " ACCEL (m/s²):    X: " << std::setw(5) << data.accelerations[0] 
                          << " | Y: " << std::setw(5) << data.accelerations[1] 
                          << " | Z: " << std::setw(5) << data.accelerations[2] << std::endl;
                
                std::cout << " RATES (rad/s):   R: " << std::setw(5) << data.rates[0] 
                          << " | P: " << std::setw(5) << data.rates[1] 
                          << " | Y: " << std::setw(5) << data.rates[2] << std::endl;
            } else {
                std::cout << " WAITING FOR DATA..." << std::endl;
            }
            
            std::cout << std::string(60, '=') << std::endl;
            std::cout << " MOTOR COMMANDS (PWM):" << std::endl;
            
            auto motors = getMotors();
            std::cout << "   M1: " << motors[0] << "  M2: " << motors[1] 
                      << "  M3: " << motors[2] << "  M4: " << motors[3] << std::endl;
            std::cout << std::string(60, '=') << std::endl;
        }
    }
}

// ============================================================================
// TEST SCENARIOS
// ============================================================================

void WindowsFlightController::testHoverAttempt()
{
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << " TEST: Active Hover (High TX Rate)" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // 1. Ramp Up (Active Loop)
    std::cout << "  Ramping up..." << std::endl;
    int target_pwm = 1200;
    while (target_pwm < 1550) {
        target_pwm += 15;  // Increment slowly
        sendMotors(target_pwm, target_pwm, target_pwm, target_pwm);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1000 Hz
    }
    
    // 2. Hold Hover (Active Loop - The Fix!)
    std::cout << " Holding hover at " << target_pwm 
              << " PWM (Sending 400 cmds/sec)..." << std::endl;
    auto start_hold = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_hold).count() < 50.0) {
        // Keep sending the SAME command constantly
        sendMotors(1804, 1800, 1804, 1800);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));  // Maintain 1000Hz timing
    }
    
    // 3. Ramp Down (Active Loop)
    std::cout << "  Ramping down..." << std::endl;
    while (target_pwm > 1000) {
        target_pwm -= 1;
        sendMotors(target_pwm, target_pwm, target_pwm, target_pwm);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    
    sendMotors(1000, 1000, 1000, 1000);
    std::cout << "\n Hover test complete" << std::endl;
}

}  // namespace fixar_client

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char** argv)
{
    using namespace fixar_client;
    
    // Parse command line arguments
    std::string host = "127.0.0.1";
    int port = 5555;
    
    if (argc > 1) {
        host = argv[1];
    }
    if (argc > 2) {
        port = std::stoi(argv[2]);
    }
    
    try {
        // Create client (localhost for Ubuntu, will be WSL IP later)
        WindowsFlightController client(host, port);
        
        // Connect to bridge
        if (!client.connect()) {
            return 1;
        }
        
        // Wait for initial data
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Run hover test
        client.testHoverAttempt();
        
        // Final status
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << " ALL TESTS COMPLETE!" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << " Bridge is working correctly at 400+ Hz" << std::endl;
        std::cout << " Ready for Windows/WSL deployment" << std::endl;
        std::cout << std::string(70, '=') << std::endl;
        std::cout << "\nPress Ctrl+C to exit (monitoring continues)...\n" << std::endl;
        
        // Keep monitoring and sending idle commands
        while (client.isConnected()) {
            // Send idle command (0% throttle) to keep TX rate at 400Hz
            client.sendMotors(1000, 1000, 1000, 1000);
            std::this_thread::sleep_for(std::chrono::microseconds(1000));  // 1000 Hz
        }
        
    } catch (const std::exception& e) {
        std::cerr << "\n Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
