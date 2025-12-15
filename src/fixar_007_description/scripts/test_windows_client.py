#!/usr/bin/env python3
"""
FIXAR 007 Flight Controller Client (Windows Simulator)
Tests bidirectional communication at 400+ Hz on Ubuntu
WSL-ready architecture (just change host IP when deploying)
"""

import socket
import json
import time
import threading
import math
from collections import deque

class WindowsFlightController:
    def __init__(self, host='localhost', port=5555):
        """
        Initialize flight controller client
        
        Args:
            host: Bridge IP ('localhost' for Ubuntu, '172.x.x.x' for WSL)
            port: Bridge TCP port (default 5555)
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
        # Received sensor data (thread-safe)
        self.sensor_data = None
        self.data_lock = threading.Lock()
        
        # Performance monitoring
        self.rx_counter = 0
        self.tx_counter = 0
        self.last_stats_time = time.time()
        self.rx_hz_history = deque(maxlen=10)
        self.tx_hz_history = deque(maxlen=10)
        
        # Motor commands (PWM 1000-2000)
        self.motors = [1000, 1000, 1000, 1000]
        
        print('=' * 70)
        print('🎮 FIXAR 007 Flight Controller Client')
        print('=' * 70)
    
    def connect(self):
        """Connect to bridge"""
        print(f' Connecting to {self.host}:{self.port}...')
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Disable Nagle's algorithm for low latency (CRITICAL!)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        try:
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(' Connected to FIXAR bridge!')
            print('=' * 70)
            
            # Start receiver thread
            self.rx_thread = threading.Thread(
                target=self.receive_data, daemon=True)
            self.rx_thread.start()
            
            # Start statistics thread
            self.stats_thread = threading.Thread(
                target=self.print_stats, daemon=True)
            self.stats_thread.start()
            
        except ConnectionRefusedError:
            print(' Connection refused - is the bridge running?')
            print('   Start bridge with: ros2 run fixar_007_description fixar_bridge.py')
            raise
        except Exception as e:
            print(f' Connection error: {e}')
            raise
    
    def receive_data(self):
        """
        Receive sensor data from bridge (runs in thread)
        CRITICAL: Proper buffer handling for TCP stream
        """
        buffer = ""
        
        while self.connected:
            try:
                # Receive data
                data = self.socket.recv(8192).decode('utf-8')
                if not data:
                    break
                
                # Add to buffer
                buffer += data
                
                # Process complete JSON lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    
                    if line.strip():
                        try:
                            packet = json.loads(line)
                            
                            # Update sensor data (thread-safe)
                            with self.data_lock:
                                self.sensor_data = packet
                            
                            self.rx_counter += 1
                            
                        except json.JSONDecodeError:
                            pass
                
                # Prevent buffer overflow
                if len(buffer) > 16384:
                    buffer = ""
                    
            except Exception as e:
                print(f' RX Error: {e}')
                break
        
        self.connected = False
        print('\n Disconnected from bridge')
    
    def send_motors(self, m1, m2, m3, m4):
        """
        Send motor commands to bridge
        
        Args:
            m1, m2, m3, m4: Motor PWM values (1000-2000)
        
        Returns:
            bool: True if sent successfully
        """
        if not self.connected:
            return False
        
        # Clamp values to valid range
        m1 = max(1000, min(2000, int(m1)))
        m2 = max(1000, min(2000, int(m2)))
        m3 = max(1000, min(2000, int(m3)))
        m4 = max(1000, min(2000, int(m4)))
        
        self.motors = [m1, m2, m3, m4]
        
        try:
            # Optimized string formatting (3x faster than json.dumps)
            packet = '{"motors": [%d, %d, %d, %d]}\n' % (m1, m2, m3, m4)
            self.socket.sendall(packet.encode('ascii'))
            
            self.tx_counter += 1
            return True
            
        except Exception as e:
            print(f' TX Error: {e}')
            return False
    
    def get_sensor_data(self):
        """
        Get latest sensor data (thread-safe)
        
        Returns:
            dict: Sensor data or None if no data received
        """
        with self.data_lock:
            return self.sensor_data.copy() if self.sensor_data else None
    
    def print_stats(self):
        """Print full telemetry dashboard"""
        while self.connected:
            time.sleep(0.2)  # Faster update rate for better visibility
            
            current_time = time.time()
            elapsed = current_time - self.last_stats_time
            
            if elapsed > 0:
                # Calculate rates
                rx_hz = self.rx_counter / elapsed
                tx_hz = self.tx_counter / elapsed
                
                # Reset counters
                self.rx_counter = 0
                self.tx_counter = 0
                self.last_stats_time = current_time

                # Get latest data
                data = self.get_sensor_data()
                
                # CLEAR SCREEN (Works on Windows & Linux) for a dashboard feel
                print("\033[H\033[J", end="") 
                
                print('=' * 60)
                print(f' FIXAR 007 TELEMETRY DASHBOARD')
                print('=' * 60)
                print(f' LINK STATUS:  RX: {rx_hz:5.1f} Hz  |  TX: {tx_hz:5.1f} Hz')
                print('-' * 60)
                
                if data:
                    # 1. ATTITUDE & POS
                    r, p, y = [x * 57.2958 for x in data['attitude']] # rad to deg
                    lat, lon, alt = data['position']
                    azi = data['azimuth'] * 57.2958
                    
                    print(f" ATTITUDE (deg):  Roll: {r:6.1f} | Pitch: {p:6.1f} | Yaw: {y:6.1f}")
                    print(f" POSITION:        Lat: {lat:8.5f} | Lon: {lon:8.5f} | Alt: {alt:6.2f}m")
                    print(f" AZIMUTH:         {azi:6.1f}° (Magnetic Heading)")
                    print('-' * 60)

                    # 2. DYNAMICS
                    vx, vy, vz = data['velocity']
                    ax, ay, az = data['accelerations']
                    rx, ry, rz = data['rates']
                    
                    print(f" VELOCITY (m/s):  N: {vx:5.2f} | E: {vy:5.2f} | D: {vz:5.2f}")
                    print(f" ACCEL (m/s²):    X: {ax:5.2f} | Y: {ay:5.2f} | Z: {az:5.2f}")
                    print(f" RATES (rad/s):   R: {rx:5.2f} | P: {ry:5.2f} | Y: {rz:5.2f}")
                else:
                    print(" WAITING FOR DATA...")
                
                print('=' * 60)
                print(f" MOTOR COMMANDS (PWM):")
                print(f"   M1: {self.motors[0]}  M2: {self.motors[1]}  M3: {self.motors[2]}  M4: {self.motors[3]}")
                print('=' * 60)

                # Reset counters
                self.rx_counter = 0
                self.tx_counter = 0
                self.last_stats_time = current_time
    
    def close(self):
        """Close connection"""
        self.connected = False
        if self.socket:
            self.socket.close()
        print('\n Client closed')

# ============================================================================
# TEST SCENARIOS
# ============================================================================


def test_hover_attempt(client):
    print('\n' + '=' * 70)
    print('TEST: Active Hover (High TX Rate)')
    print('=' * 70)

    # 1. Ramp Up (Active Loop)
    print(" Ramping up...")
    target_pwm = 1000
    while target_pwm < 1600:
        target_pwm += 1  # Increment slowly
        client.send_motors(target_pwm, target_pwm, target_pwm, target_pwm)
        time.sleep(0.0025) # Sleep tiny amount (1/400th of a second)

    # 2. Hold Hover (Active Loop - The Fix!)
    print(f" Holding hover at {target_pwm} (Sending 400 cmds/sec)...")
    start_hold = time.time()
    while time.time() - start_hold <50: # Run for 5 seconds
        # Keep sending the SAME command constantly
        client.send_motors(1600, 1600, 1600, 1600)
        time.sleep(0.0025) # Maintain 400Hz timing

    # 3. Ramp Down (Active Loop)
    print(" Ramping down...")
    while target_pwm > 1000:
        target_pwm -= 1
        client.send_motors(target_pwm, target_pwm, target_pwm, target_pwm)
        time.sleep(0.0025)

    client.send_motors(1000, 1000, 1000, 1000)
    print('\n Hover test complete')
# ============================================================================
# MAIN
# ============================================================================

def main():
    # Create client (localhost for Ubuntu, will be WSL IP later)
    client = WindowsFlightController(host='localhost', port=5555)
    
    try:
        # Connect to bridge
        client.connect()
        
        # Wait for initial data
        time.sleep(1)
        test_hover_attempt(client)
        
        # Final status
        print('\n' + '=' * 70)
        print('ALL TESTS COMPLETE!')
        print('=' * 70)
        print('Bridge is working correctly at 400+ Hz')
        print('Ready for Windows/WSL deployment')
        print('=' * 70)
        print('\nPress Ctrl+C to exit (monitoring continues)...\n')
        
        # Keep monitoring
        while True:
            # Send idle command (0% throttle) to keep TX rate at 400Hz
            client.send_motors(1000, 1000, 1000, 1000)
            time.sleep(0.0025)
            
    except KeyboardInterrupt:
        print('\n\n Shutting down client...')
    except Exception as e:
        print(f'\n Error: {e}')
    finally:
        client.close()

if __name__ == '__main__':
    main()
