/**
 * FIXAR 007 High-Performance Windows Bridge
 * Production-ready bidirectional communication at 400+ Hz
 * C++ implementation for maximum performance
 */

#ifndef FIXAR_WINDOWS_BRIDGE_HPP_
#define FIXAR_WINDOWS_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <actuator_msgs/msg/actuators.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <deque>
#include <array>
#include <string>
#include <memory>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace fixar_bridge {

/**
 * Thread-safe sensor data structure
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
 * Thread-safe queue for motor commands
 */
template<typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_size = 10) : max_size_(max_size) {}
    
    bool try_push(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            // Drop oldest if full
            queue_.pop();
        }
        queue_.push(value);
        return true;
    }
    
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = queue_.front();
        queue_.pop();
        return true;
    }
    
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    size_t max_size_;
};

/**
 * Main bridge node class
 */
class FixarWindowsBridge : public rclcpp::Node {
public:
    FixarWindowsBridge();
    ~FixarWindowsBridge();

private:
    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    static constexpr int TCP_PORT = 5555;
    static constexpr int TARGET_TX_HZ = 1000;
    static constexpr double MAX_ROT_VELOCITY = 900.0;  // rad/s
    double vertical_velocity_ = 0.0;
    std::chrono::steady_clock::time_point last_imu_time_;
    double last_vertical_accel_ = 0.0;

   
    double last_altitude_ = 0.0;
    double last_gps_time_ = 0.0;
    
    // ========================================================================
    // SENSOR DATA (Thread-safe)
    // ========================================================================
    SensorData sensor_data_;
    mutable std::mutex sensor_mutex_;
    
    // ========================================================================
    // MOTOR COMMAND QUEUE
    // ========================================================================
    ThreadSafeQueue<std::array<int, 4>> motor_queue_;
    std::array<int, 4> last_motor_cmd_;
    std::mutex motor_cmd_mutex_;
    
    // ========================================================================
    // ROS2 SUBSCRIBERS
    // ========================================================================
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ========================================================================
    // ROS2 PUBLISHER
    // ========================================================================
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_pub_;
    
    // ========================================================================
    // PERFORMANCE MONITORING
    // ========================================================================
    std::atomic<uint64_t> tx_counter_;
    std::atomic<uint64_t> rx_counter_;
    std::chrono::steady_clock::time_point last_stats_time_;
    std::deque<double> tx_hz_history_;
    std::deque<double> rx_hz_history_;
    std::mutex stats_mutex_;
    
    // ========================================================================
    // TCP SERVER
    // ========================================================================
    int server_socket_;
    int client_socket_;
    std::atomic<bool> client_connected_;
    std::atomic<bool> shutdown_requested_;
    
    std::unique_ptr<std::thread> server_thread_;
    std::unique_ptr<std::thread> motor_thread_;
    
    // ========================================================================
    // TIMERS
    // ========================================================================
    rclcpp::TimerBase::SharedPtr tx_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // ========================================================================
    // COORDINATE CONVERSION
    // ========================================================================
    void quaternionToEuler(double x, double y, double z, double w,
                          double& roll, double& pitch, double& yaw);
    double pwmToRadS(int pwm);
    
    // ========================================================================
    // ROS2 CALLBACKS
    // ========================================================================
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    void baroCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // ========================================================================
    // MOTOR COMMAND PROCESSING
    // ========================================================================
    void motorProcessor();
    
    // ========================================================================
    // TCP SERVER
    // ========================================================================
    void setupTcpServer();
    void acceptClients();
    void handleClientRx(int client_socket);
    
    // ========================================================================
    // SENSOR DATA TRANSMISSION
    // ========================================================================
    void sendSensorData();
    std::string sensorDataToJson(const SensorData& data);
    
    // ========================================================================
    // PERFORMANCE MONITORING
    // ========================================================================
    void printStatistics();
};

}  // namespace fixar_bridge

#endif  // FIXAR_WINDOWS_BRIDGE_HPP_
