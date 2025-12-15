#!/usr/bin/env python3
"""
FIXAR 007 High-Performance Windows Bridge
Production-ready bidirectional communication at 400+ Hz
For Ubuntu testing (WSL-ready architecture)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, MagneticField, FluidPressure
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
import socket
import json
import threading
import time
import queue
from collections import deque
import math

class FixarWindowsBridge(Node):
    def __init__(self):
        super().__init__('fixar_windows_bridge')
        
        # ====================================================================
        # CONFIGURATION
        # ====================================================================
        self.TCP_PORT = 5555
        self.TARGET_TX_HZ = 400  # Sensor data transmission rate
        self.MAX_ROT_VELOCITY = 900.0  # rad/s from SDF maxRotVelocity
        
        # ====================================================================
        # SENSOR DATA STORAGE (Thread-safe)
        # ====================================================================
        self.sensor_lock = threading.Lock()
        self.sensor_data = {
            'rates': [0.0, 0.0, 0.0],           # rad/s (p, q, r)
            'accelerations': [0.0, 0.0, 0.0],   # m/s² (ax, ay, az)
            'attitude': [0.0, 0.0, 0.0],        # rad (roll, pitch, yaw)
            'position': [0.0, 0.0, 0.0],        # [lat°, lon°, alt_m]
            'velocity': [0.0, 0.0, 0.0],        # m/s (vN, vE, vD)
            'azimuth': 0.0,                     # rad (magnetic heading)
            'timestamp': 0.0
        }
        
        # ====================================================================
        # MOTOR COMMAND QUEUE (Thread-safe)
        # ====================================================================
        self.motor_queue = queue.Queue(maxsize=10)
        self.last_motor_cmd = [1000, 1000, 1000, 1000]  # PWM values
        
        # ====================================================================
        # ROS2 SUBSCRIBERS
        # ====================================================================
        self.imu_sub = self.create_subscription(
            Imu, '/fixar_007/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fixar_007/gps', self.gps_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/fixar_007/magnetometer', self.mag_callback, 10)
        self.baro_sub = self.create_subscription(
            FluidPressure, '/fixar_007/air_pressure', self.baro_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/fixar_007/odometry', self.odom_callback, 10)
        
        # ====================================================================
        # ROS2 PUBLISHER
        # ====================================================================
        self.motor_pub = self.create_publisher(
            Actuators, '/fixar_007/command/motor_speed', 10)
        
        # ====================================================================
        # PERFORMANCE MONITORING
        # ====================================================================
        self.tx_counter = 0
        self.rx_counter = 0
        self.last_stats_time = time.time()
        self.tx_hz_history = deque(maxlen=10)
        self.rx_hz_history = deque(maxlen=10)
        
        # ====================================================================
        # TCP SERVER SETUP
        # ====================================================================
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.TCP_PORT))  # WSL-ready
        self.server_socket.listen(1)
        
        self.client_socket = None
        self.client_connected = False
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('FIXAR 007 Windows Bridge - PRODUCTION MODE')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Listening on 0.0.0.0:{self.TCP_PORT}')
        self.get_logger().info(f'Target TX rate: {self.TARGET_TX_HZ} Hz')
        self.get_logger().info(f'Max motor speed: {self.MAX_ROT_VELOCITY} rad/s')
        self.get_logger().info('Waiting for client connection...')
        self.get_logger().info('=' * 70)
        
        # ====================================================================
        # START THREADS
        # ====================================================================
        # Thread 1: Accept client connections
        self.server_thread = threading.Thread(
            target=self.accept_clients, daemon=True)
        self.server_thread.start()
        
        # Thread 2: Process motor commands from queue
        self.motor_thread = threading.Thread(
            target=self.motor_processor, daemon=True)
        self.motor_thread.start()
        
        # ====================================================================
        # HIGH-FREQUENCY TIMERS
        # ====================================================================
        # Timer 1: Transmit sensor data at 400 Hz
        timer_period = 1.0 / self.TARGET_TX_HZ
        self.tx_timer = self.create_timer(timer_period, self.send_sensor_data)
        
        # Timer 2: Print statistics at 1 Hz
        self.stats_timer = self.create_timer(1.0, self.print_statistics)
    
    # ========================================================================
    # COORDINATE CONVERSION UTILITIES
    # ========================================================================
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw) in radians"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def pwm_to_rad_s(self, pwm):
        """
        Convert PWM (1000-2000) to rad/s for motor
        PWM 1000 = 0 rad/s (motor off)
        PWM 2000 = MAX_ROT_VELOCITY rad/s (full throttle)
        """
        pwm = max(1000, min(2000, pwm))  # Clamp to valid range
        normalized = (pwm - 1000) / 1000.0  # 0.0 to 1.0
        return normalized * self.MAX_ROT_VELOCITY
    
    # ========================================================================
    # ROS2 CALLBACKS (Fast, lock-protected)
    # ========================================================================
    
    def imu_callback(self, msg):
        """Process IMU data: rates, accelerations, attitude"""
        with self.sensor_lock:
            # Angular velocity (body rates)
            self.sensor_data['rates'] = [
                msg.angular_velocity.x,  # p (roll rate)
                msg.angular_velocity.y,  # q (pitch rate)
                msg.angular_velocity.z   # r (yaw rate)
            ]
            
            # Linear acceleration
            self.sensor_data['accelerations'] = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
            
            # Attitude from quaternion
            roll, pitch, yaw = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            self.sensor_data['attitude'] = [roll, pitch, yaw]
            self.sensor_data['timestamp'] = time.time()
    
    def gps_callback(self, msg):
        """Process GPS data: latitude, longitude, altitude"""
        with self.sensor_lock:
            self.sensor_data['position'] = [
                msg.latitude,
                msg.longitude,
                msg.altitude
            ]
    
    def mag_callback(self, msg):
        """Process magnetometer data: calculate azimuth"""
        with self.sensor_lock:
            # Calculate azimuth (magnetic heading) from magnetic field
            azimuth = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
            self.sensor_data['azimuth'] = azimuth
    
    def odom_callback(self, msg):
        """Process odometry data: velocity in NED frame"""
        with self.sensor_lock:
            # Convert ENU (Gazebo) to NED (aviation standard)
            self.sensor_data['velocity'] = [
                msg.twist.twist.linear.x,   # North (forward)
                msg.twist.twist.linear.y,   # East (right)
                -msg.twist.twist.linear.z   # Down (negate up)
            ]
    
    def baro_callback(self, msg):
        """Process barometer data (optional enhancement)"""
        # Could calculate pressure altitude here if needed
        pass
    
    # ========================================================================
    # MOTOR COMMAND PROCESSING
    # ========================================================================
    
    def motor_processor(self):
        """
        Process motor commands from queue and publish to ROS2
        Runs in separate thread to avoid blocking TCP receiver
        """
        while True:
            try:
                # Get motor command from queue (blocking with timeout)
                pwm_commands = self.motor_queue.get(timeout=0.1)
                
                # Convert PWM (1000-2000) to rad/s
                motor_velocities = [
                    self.pwm_to_rad_s(pwm) for pwm in pwm_commands
                ]
                
                # Publish to ROS2
                msg = Actuators()
                msg.velocity = motor_velocities
                self.motor_pub.publish(msg)
                
                # Update last command and counter
                self.last_motor_cmd = pwm_commands
                self.rx_counter += 1
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Motor processor error: {e}')
    
    # ========================================================================
    # TCP SERVER (Client Management)
    # ========================================================================
    
    def accept_clients(self):
        """Accept incoming client connections (blocking)"""
        while True:
            try:
                client_socket, address = self.server_socket.accept()
                
                # Disable Nagle's algorithm for low latency
                client_socket.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                self.client_socket = client_socket
                self.client_connected = True
                
                self.get_logger().info('=' * 70)
                self.get_logger().info(f'CLIENT CONNECTED: {address[0]}:{address[1]}')
                self.get_logger().info('=' * 70)
                
                # Handle this client's incoming data
                self.handle_client_rx(client_socket)
                
            except Exception as e:
                self.get_logger().error(f'Accept error: {e}')
    
    def handle_client_rx(self, client_socket):
        """
        Handle incoming motor commands from client
        CRITICAL: Proper buffer handling for TCP stream
        """
        buffer = ""
        
        while self.client_connected:
            try:
                # Receive data
                data = client_socket.recv(4096)
                if not data:
                    break
                
                # Add to buffer
                buffer += data.decode('utf-8')
                
                # Process complete JSON lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    if line.strip():
                        try:
                            # Parse JSON motor command
                            cmd = json.loads(line)
                            
                            if 'motors' in cmd and len(cmd['motors']) == 4:
                                # Add to queue (non-blocking)
                                try:
                                    self.motor_queue.put_nowait(cmd['motors'])
                                except queue.Full:
                                    # Queue full - drop oldest and add new
                                    try:
                                        self.motor_queue.get_nowait()
                                        self.motor_queue.put_nowait(cmd['motors'])
                                    except:
                                        pass
                        
                        except json.JSONDecodeError as e:
                            self.get_logger().warn(f'JSON decode error: {e}')
                
                # Prevent buffer overflow
                if len(buffer) > 8192:
                    self.get_logger().warn('Buffer overflow, clearing')
                    buffer = ""
                
            except Exception as e:
                self.get_logger().error(f'RX error: {e}')
                break
        
        # Client disconnected
        self.client_connected = False
        client_socket.close()
        self.get_logger().info('=' * 70)
        self.get_logger().info('CLIENT DISCONNECTED')
        self.get_logger().info('=' * 70)
    
    # ========================================================================
    # SENSOR DATA TRANSMISSION (400 Hz Timer)
    # ========================================================================
    
    def send_sensor_data(self):
        """Send sensor data to client at 400 Hz"""
        if not self.client_connected or self.client_socket is None:
            return
        
        try:
            # Get sensor snapshot (fast lock)
            with self.sensor_lock:
                data_snapshot = self.sensor_data.copy()
            
            # Create JSON packet (newline-delimited)
            packet = json.dumps(data_snapshot) + '\n'
            
            # Send to client
            self.client_socket.sendall(packet.encode('utf-8'))
            
            self.tx_counter += 1
            
        except BrokenPipeError:
            self.client_connected = False
            self.get_logger().warn('Client disconnected (broken pipe)')
        except Exception as e:
            self.client_connected = False
            self.get_logger().error(f'TX error: {e}')
    
    # ========================================================================
    # PERFORMANCE MONITORING
    # ========================================================================
    
    def print_statistics(self):
        """Print performance statistics every second"""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time
        
        if elapsed > 0:
            # Calculate rates
            tx_hz = self.tx_counter / elapsed
            rx_hz = self.rx_counter / elapsed
            
            # Update history
            self.tx_hz_history.append(tx_hz)
            self.rx_hz_history.append(rx_hz)
            
            # Calculate averages
            avg_tx_hz = sum(self.tx_hz_history) / len(self.tx_hz_history)
            avg_rx_hz = sum(self.rx_hz_history) / len(self.rx_hz_history)
            
            # Print status
            if self.client_connected:
                self.get_logger().info(
                    f' TX: {tx_hz:6.1f} Hz (avg: {avg_tx_hz:6.1f}) | '
                    f'RX: {rx_hz:6.1f} Hz (avg: {avg_rx_hz:6.1f}) | '
                    f'Motors: [{self.last_motor_cmd[0]:4d}, {self.last_motor_cmd[1]:4d}, '
                    f'{self.last_motor_cmd[2]:4d}, {self.last_motor_cmd[3]:4d}]'
                )
            
            # Reset counters
            self.tx_counter = 0
            self.rx_counter = 0
            self.last_stats_time = current_time

def main(args=None):
    rclpy.init(args=args)
    bridge = FixarWindowsBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('\n Shutting down bridge...')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
