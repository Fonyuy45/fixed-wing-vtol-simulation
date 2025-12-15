/**
 * FIXAR 007 High-Performance Windows Bridge
 * C++ Implementation
 */

#include "fixar_007_description/fixar_windows_bridge.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <sstream>
#include <iomanip>

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace fixar_bridge {

FixarWindowsBridge::FixarWindowsBridge()
    : Node("fixar_windows_bridge")
    , motor_queue_(10)
    , last_motor_cmd_{1000, 1000, 1000, 1000}
    , tx_counter_(0)
    , rx_counter_(0)
    , server_socket_(-1)
    , client_socket_(-1)
    , client_connected_(false)
    , shutdown_requested_(false)
{
    // ========================================================================
    // INITIALIZE SENSOR DATA
    // ========================================================================
    sensor_data_.timestamp = 0.0;
    last_stats_time_ = std::chrono::steady_clock::now();
    
    // ========================================================================
    // ROS2 SUBSCRIBERS
    // ========================================================================
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/fixar_007/imu", 10,
        std::bind(&FixarWindowsBridge::imuCallback, this, std::placeholders::_1));
    
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/fixar_007/gps", 10,
        std::bind(&FixarWindowsBridge::gpsCallback, this, std::placeholders::_1));
    
    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "/fixar_007/magnetometer", 10,
        std::bind(&FixarWindowsBridge::magCallback, this, std::placeholders::_1));
    
    baro_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/fixar_007/air_pressure", 10,
        std::bind(&FixarWindowsBridge::baroCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fixar_007/odometry", 10,
        std::bind(&FixarWindowsBridge::odomCallback, this, std::placeholders::_1));
    
    // ========================================================================
    // ROS2 PUBLISHER
    // ========================================================================
    motor_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(
        "/fixar_007/command/motor_speed", 10);
    
    // ========================================================================
    // TCP SERVER SETUP
    // ========================================================================
    setupTcpServer();
    
    RCLCPP_INFO(this->get_logger(), "======================================================================");
    RCLCPP_INFO(this->get_logger(), "FIXAR 007 Windows Bridge - C++ PRODUCTION MODE");
    RCLCPP_INFO(this->get_logger(), "======================================================================");
    RCLCPP_INFO(this->get_logger(), "Listening on 0.0.0.0:%d", TCP_PORT);
    RCLCPP_INFO(this->get_logger(), "Target TX rate: %d Hz", TARGET_TX_HZ);
    RCLCPP_INFO(this->get_logger(), "Max motor speed: %.1f rad/s", MAX_ROT_VELOCITY);
    RCLCPP_INFO(this->get_logger(), "Waiting for client connection...");
    RCLCPP_INFO(this->get_logger(), "======================================================================");
    
    // ========================================================================
    // START THREADS
    // ========================================================================
    server_thread_ = std::make_unique<std::thread>(
        &FixarWindowsBridge::acceptClients, this);
    
    motor_thread_ = std::make_unique<std::thread>(
        &FixarWindowsBridge::motorProcessor, this);
    
    // ========================================================================
    // HIGH-FREQUENCY TIMERS
    // ========================================================================
    auto timer_period = std::chrono::microseconds(1000000 / TARGET_TX_HZ);
    tx_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&FixarWindowsBridge::sendSensorData, this));
    
    stats_timer_ = this->create_wall_timer(
        1s,
        std::bind(&FixarWindowsBridge::printStatistics, this));
}

FixarWindowsBridge::~FixarWindowsBridge()
{
    shutdown_requested_ = true;
    
    // FORCE WAKE UP: Shutdown breaks blocking calls immediately
    if (client_socket_ >= 0) {
        shutdown(client_socket_, SHUT_RDWR);
        close(client_socket_);
    }
    if (server_socket_ >= 0) {
        shutdown(server_socket_, SHUT_RDWR);
        close(server_socket_);
    }
    
    // Join threads
    if (server_thread_ && server_thread_->joinable()) {
        server_thread_->join();
    }
    if (motor_thread_ && motor_thread_->joinable()) {
        motor_thread_->join();
    }
    
    RCLCPP_INFO(this->get_logger(), "Bridge shutdown complete");
}

// ============================================================================
// COORDINATE CONVERSION
// ============================================================================

void FixarWindowsBridge::quaternionToEuler(
    double x, double y, double z, double w,
    double& roll, double& pitch, double& yaw)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    sinp = std::max(-1.0, std::min(1.0, sinp));
    pitch = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

double FixarWindowsBridge::pwmToRadS(int pwm)
{
    // Clamp PWM to valid range
    pwm = std::max(1000, std::min(2000, pwm));
    
    // Normalize to 0.0 - 1.0
    double normalized = (pwm - 1000) / 1000.0;
    
    // Scale to motor velocity
    return normalized * MAX_ROT_VELOCITY;
}

// ============================================================================
// ROS2 CALLBACKS
// ============================================================================

void FixarWindowsBridge::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // Angular velocity (body rates)
    sensor_data_.rates[0] = msg->angular_velocity.x;  // p (roll rate)
    sensor_data_.rates[1] = msg->angular_velocity.y;  // q (pitch rate)
    sensor_data_.rates[2] = msg->angular_velocity.z;  // r (yaw rate)
    
    // Linear acceleration
    sensor_data_.accelerations[0] = msg->linear_acceleration.x;
    sensor_data_.accelerations[1] = msg->linear_acceleration.y;
    sensor_data_.accelerations[2] = msg->linear_acceleration.z;
    
    // Attitude from quaternion
    double roll, pitch, yaw;
    quaternionToEuler(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w,
        roll, pitch, yaw
    );
    sensor_data_.attitude[0] = roll;
    sensor_data_.attitude[1] = pitch;
    sensor_data_.attitude[2] = yaw;
    
    // Timestamp
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    sensor_data_.timestamp = std::chrono::duration<double>(duration).count();
}

void FixarWindowsBridge::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // Static variables for state between loops
    static double last_alt = 0.0;
    static double last_time = 0.0;
    static double smoothed_vz = 0.0; // Filtered vertical speed
    
    double current_time = this->now().seconds();
    double dt = current_time - last_time;

    // Update Position
    sensor_data_.position[0] = msg->latitude;
    sensor_data_.position[1] = msg->longitude;
    sensor_data_.position[2] = msg->altitude;

    // === NOISE FILTERING LOGIC ===
    if (dt > 0.0 && last_time > 0.0) {
        // 1. Calculate Raw (Noisy) Velocity
        double dz = msg->altitude - last_alt;
        double raw_vz = dz / dt;
        
        // 2. Apply Low-Pass Filter (Alpha = 0.95)
        // This ignores 95% of sudden spikes and trusts the history
        double alpha = 0.98;
        smoothed_vz = (alpha * smoothed_vz) + ((1.0 - alpha) * raw_vz);
        
        // 3. Update Output (NED Down = Negative Up)
        //sensor_data_.velocity[2] = -smoothed_vz;
    }

    last_alt = msg->altitude;
    last_time = current_time;
}

/*
void FixarWindowsBridge::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    sensor_data_.position[0] = msg->latitude;
    sensor_data_.position[1] = msg->longitude;
    sensor_data_.position[2] = msg->altitude;
}

*/

void FixarWindowsBridge::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // Calculate azimuth (magnetic heading) from magnetic field
    sensor_data_.azimuth = std::atan2(
        msg->magnetic_field.y,
        msg->magnetic_field.x
    );
}

void FixarWindowsBridge::baroCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
{
    // Optional: Could calculate pressure altitude here if needed
    (void)msg;  // Suppress unused warning
}

/*
void FixarWindowsBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // Convert ENU (Gazebo) to NED (aviation standard)
    sensor_data_.velocity[0] = msg->twist.twist.linear.x;   // North (forward)
    sensor_data_.velocity[1] = msg->twist.twist.linear.y;   // East (right)
    //sensor_data_.velocity[2] = -msg->twist.twist.linear.z;  // Down (negate up)
}
*/

void FixarWindowsBridge::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // 1. Get raw Body velocities
    double vx_body = msg->twist.twist.linear.x;  // Forward
    double vy_body = msg->twist.twist.linear.y;  // Left (ROS standard)
    double vz_body = msg->twist.twist.linear.z;  // Up (ROS standard)
    
    // 2. Get Attitude (Calculated in imuCallback)
    double roll  = sensor_data_.attitude[0];
    double pitch = sensor_data_.attitude[1];
    double yaw   = sensor_data_.attitude[2];
    
    // 3. Pre-calculate Trig
    double c_r = cos(roll);
    double s_r = sin(roll);
    double c_p = cos(pitch);
    double s_p = sin(pitch);
    double c_y = cos(yaw);
    double s_y = sin(yaw);
    
    // 4. Rotate Body to World (NED)
    // Formula for North (X)
    double v_north = (c_p * c_y) * vx_body + 
                     (s_r * s_p * c_y - c_r * s_y) * vy_body + 
                     (c_r * s_p * c_y + s_r * s_y) * vz_body;

    // Formula for East (Y)
    double v_east = (c_p * s_y) * vx_body + 
                    (s_r * s_p * s_y + c_r * c_y) * vy_body + 
                    (c_r * s_p * s_y - s_r * c_y) * vz_body;

    // Formula for Down (Z) - The Critical Fix!
    // If you pitch down (-pitch), Forward Speed (vx) contributes to Down Velocity.
    double v_down = (-s_p) * vx_body + 
                    (s_r * c_p) * vy_body + 
                    (c_r * c_p) * vz_body;
                    
    // 5. Save to Data Structure
    sensor_data_.velocity[0] = v_north;
    sensor_data_.velocity[1] = v_east;
    // We invert v_down here because typical NED "Down" is positive, 
    // but the formula above might yield negative for "Earth Down" depending on the input frame.
    // Standard aeronautics: Positive D = Down towards earth.
    // If your drone pitches down (negative pitch), -sin(-val) = positive. So v_down is positive. 
    // This is correct.
    sensor_data_.velocity[2] = v_down;
}

// ============================================================================
// MOTOR COMMAND PROCESSING
// ============================================================================


void FixarWindowsBridge::motorProcessor()
{
    std::array<int, 4> pwm_commands;
    
    while (!shutdown_requested_) {
        // Try to get motor command from queue
        if (motor_queue_.try_pop(pwm_commands)) {
            // Convert PWM to rad/s
            auto msg = actuator_msgs::msg::Actuators();
            msg.velocity.resize(4);
            
            for (size_t i = 0; i < 4; ++i) {
                msg.velocity[i] = pwmToRadS(pwm_commands[i]);
            }
            
            // Publish to ROS2
            motor_pub_->publish(msg);
            
            // Update last command and counter
            {
                std::lock_guard<std::mutex> lock(motor_cmd_mutex_);
                last_motor_cmd_ = pwm_commands;
            }
            rx_counter_++;
        } else {
            // No commands available, sleep briefly
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
}

// ============================================================================
// TCP SERVER
// ============================================================================

void FixarWindowsBridge::setupTcpServer()
{
    // Create socket
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        throw std::runtime_error("Socket creation failed");
    }
    
    // Set socket options
    int opt = 1;
    if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR");
    }
    
    // Bind to address
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(TCP_PORT);
    
    if (bind(server_socket_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
        close(server_socket_);
        throw std::runtime_error("Socket bind failed");
    }
    
    // Listen for connections
    if (listen(server_socket_, 1) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to listen on socket");
        close(server_socket_);
        throw std::runtime_error("Socket listen failed");
    }
}

void FixarWindowsBridge::acceptClients()
{
    while (!shutdown_requested_) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(server_socket_, &readfds);

        // Set a timeout of 1 second
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        // Wait up to 1 second to see if a client is trying to connect
        // This is non-blocking so we can check shutdown_requested_
        int activity = select(server_socket_ + 1, &readfds, NULL, NULL, &timeout);

        if (activity < 0 && errno != EINTR) {
            // Error in select
            if (!shutdown_requested_) {
                RCLCPP_ERROR(this->get_logger(), "Select error");
            }
        }
        else if (activity > 0) {
            if (FD_ISSET(server_socket_, &readfds)) {
                // WE HAVE A CLIENT! Accept immediately (won't block now)
                struct sockaddr_in client_address;
                socklen_t client_len = sizeof(client_address);
                
                int client_socket = accept(
                    server_socket_,
                    (struct sockaddr*)&client_address,
                    &client_len
                );

                if (client_socket < 0) {
                    continue;
                }

                // Disable Nagle's algorithm
                int flag = 1;
                setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

                client_socket_ = client_socket;
                client_connected_ = true;

                char client_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &client_address.sin_addr, client_ip, INET_ADDRSTRLEN);

                RCLCPP_INFO(this->get_logger(), "======================================================================");
                RCLCPP_INFO(this->get_logger(), "CLIENT CONNECTED: %s:%d", 
                            client_ip, ntohs(client_address.sin_port));
                RCLCPP_INFO(this->get_logger(), "======================================================================");

                // Handle this client's incoming data (This blocks until client disconnects)
                handleClientRx(client_socket);

                // Client disconnected
                client_connected_ = false;
                close(client_socket);
                client_socket_ = -1;

                RCLCPP_INFO(this->get_logger(), "======================================================================");
                RCLCPP_INFO(this->get_logger(), "CLIENT DISCONNECTED");
                RCLCPP_INFO(this->get_logger(), "======================================================================");
            }
        }
        // If activity == 0 (Timeout), the loop repeats and checks !shutdown_requested_
    }
}

void FixarWindowsBridge::handleClientRx(int client_socket)
{
    std::string buffer;
    char recv_buffer[4096];
    
    while (client_connected_ && !shutdown_requested_) {
        // Receive data
        ssize_t bytes_received = recv(client_socket, recv_buffer, sizeof(recv_buffer), 0);
        
        if (bytes_received <= 0) {
            // Connection closed or error
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
                    auto cmd = json::parse(line);
                    
                    if (cmd.contains("motors") && cmd["motors"].is_array() && 
                        cmd["motors"].size() == 4) {
                        
                        std::array<int, 4> motors;
                        for (size_t i = 0; i < 4; ++i) {
                            motors[i] = cmd["motors"][i].get<int>();
                        }
                        
                        // Add to queue
                        motor_queue_.try_push(motors);
                    }
                } catch (const json::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "JSON parse error: %s", e.what());
                }
            }
        }
        
        // Prevent buffer overflow
        if (buffer.size() > 8192) {
            RCLCPP_WARN(this->get_logger(), "Buffer overflow, clearing");
            buffer.clear();
        }
    }
}

// ============================================================================
// SENSOR DATA TRANSMISSION
// ============================================================================

std::string FixarWindowsBridge::sensorDataToJson(const SensorData& data)
{
    json j;
    
    j["rates"] = data.rates;
    j["accelerations"] = data.accelerations;
    j["attitude"] = data.attitude;
    j["position"] = data.position;
    j["velocity"] = data.velocity;
    j["azimuth"] = data.azimuth;
    j["timestamp"] = data.timestamp;
    
    return j.dump();
}

void FixarWindowsBridge::sendSensorData()
{
    if (!client_connected_ || client_socket_ < 0) {
        return;
    }
    
    try {
        // Get sensor snapshot (fast lock)
        SensorData data_snapshot;
        {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            data_snapshot = sensor_data_;
        }
        
        // Create JSON packet (newline-delimited)
        std::string packet = sensorDataToJson(data_snapshot) + "\n";
        
        // Send to client
        ssize_t bytes_sent = send(client_socket_, packet.c_str(), packet.size(), MSG_NOSIGNAL);
        
        if (bytes_sent < 0) {
            client_connected_ = false;
            RCLCPP_WARN(this->get_logger(), "Client disconnected (send error)");
        } else {
            tx_counter_++;
        }
    } catch (const std::exception& e) {
        client_connected_ = false;
        RCLCPP_ERROR(this->get_logger(), "TX error: %s", e.what());
    }
}

// ============================================================================
// PERFORMANCE MONITORING
// ============================================================================

void FixarWindowsBridge::printStatistics()
{
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(current_time - last_stats_time_).count();
    
    if (elapsed > 0) {
        // Calculate rates
        double tx_hz = tx_counter_ / elapsed;
        double rx_hz = rx_counter_ / elapsed;
        
        // Update history
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            tx_hz_history_.push_back(tx_hz);
            rx_hz_history_.push_back(rx_hz);
            
            if (tx_hz_history_.size() > 10) {
                tx_hz_history_.pop_front();
            }
            if (rx_hz_history_.size() > 10) {
                rx_hz_history_.pop_front();
            }
            
            // Calculate averages
            double avg_tx_hz = 0.0;
            double avg_rx_hz = 0.0;
            
            for (double val : tx_hz_history_) avg_tx_hz += val;
            for (double val : rx_hz_history_) avg_rx_hz += val;
            
            avg_tx_hz /= tx_hz_history_.size();
            avg_rx_hz /= rx_hz_history_.size();
            
            // Print status
            if (client_connected_) {
                std::array<int, 4> motors;
                {
                    std::lock_guard<std::mutex> lock(motor_cmd_mutex_);
                    motors = last_motor_cmd_;
                }
                
                RCLCPP_INFO(this->get_logger(),
                    "▶ TX: %6.1f Hz (avg: %6.1f) | RX: %6.1f Hz (avg: %6.1f) | "
                    "Motors: [%4d, %4d, %4d, %4d]",
                    tx_hz, avg_tx_hz, rx_hz, avg_rx_hz,
                    motors[0], motors[1], motors[2], motors[3]
                );
            }
        }
        
        // Reset counters
        tx_counter_ = 0;
        rx_counter_ = 0;
        last_stats_time_ = current_time;
    }
}

}  // namespace fixar_bridge

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto bridge = std::make_shared<fixar_bridge::FixarWindowsBridge>();
        rclcpp::spin(bridge);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fixar_bridge"), 
                     "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
