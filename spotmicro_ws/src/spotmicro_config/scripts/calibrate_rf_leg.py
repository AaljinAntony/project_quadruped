#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, String
import sys
import termios
import tty
import threading
import datetime
import time

class LegCalibrationClient(Node):
    def __init__(self):
        super().__init__('rf_calib_node')
        self.calib_pub = self.create_publisher(Int32MultiArray, '/motor_calibration', 10)
        self.subscription = self.create_subscription(String, '/motor_status', self.status_callback, 10)
        
        # Target Leg Motors
        self.motors = [
            {'name': 'FR Foot', 'pin': 8, 'start_pwm': 224},
            {'name': 'FR Leg', 'pin': 9, 'start_pwm': 298},
            {'name': 'FR Shoulder', 'pin': 10, 'start_pwm': 306}
        ]
        self.current_idx = 0
        self.current_pwm = self.motors[self.current_idx]['start_pwm']
        self.last_status = 'Waiting for ESP32...'
        self.log_file = '/workspace/fr_leg_calibration.log'

    def status_callback(self, msg):
        self.last_status = msg.data

    def pwm_to_deg(self, pwm):
        return (pwm - 307) / 2.007127

    def initialize_legs(self):
        active_pins = [m['pin'] for m in self.motors]
        print("\nDepowering all other legs to isolate tuning...")
        for p in range(16):
            if p not in active_pins:
                msg = Int32MultiArray()
                msg.data = [int(p), 0]
                self.calib_pub.publish(msg)
                time.sleep(0.02)
                
        print("Moving target leg to its mathematical starting position...")
        for m in self.motors:
            msg = Int32MultiArray()
            msg.data = [int(m['pin']), int(m['start_pwm'])]
            self.calib_pub.publish(msg)
            time.sleep(0.1)

    def send_cmd(self, log=True):
        pin = self.motors[self.current_idx]['pin']
        msg = Int32MultiArray()
        msg.data = [int(pin), int(self.current_pwm)]
        self.calib_pub.publish(msg)
        
        if log:
            deg = self.pwm_to_deg(self.current_pwm)
            name = self.motors[self.current_idx]['name']
            with open(self.log_file, 'a') as f:
                f.write(f'[{datetime.datetime.now()}] MOTOR: {name} (Pin {pin}) -> PWM: {self.current_pwm} (Approx {deg:.2f} deg)\n')

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    try:
        rclpy.init()
        node = LegCalibrationClient()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        time.sleep(1.0) # give ROS 2 topics time to bridge
        node.initialize_legs()

        print('\nSpotMicro FR Leg Calibration (Pins 8, 9, 10)')
        print('A/F: Switch Motor | W/S: +/- 1 | E/D: +/- 10 | Q: Save & Quit')
        print('----------------------------------------------------------')

        while True:
            ch = getch().lower()
            if ch == 'q' or ch == '\x03': break
            
            if ch == 'w': node.current_pwm += 1
            elif ch == 's': node.current_pwm -= 1
            elif ch == 'e': node.current_pwm += 10
            elif ch == 'd': node.current_pwm -= 10
            elif ch == 'a': 
                node.motors[node.current_idx]['start_pwm'] = node.current_pwm
                node.current_idx = (node.current_idx - 1) % len(node.motors)
                node.current_pwm = node.motors[node.current_idx]['start_pwm']
                print(f"\n--- {node.motors[node.current_idx]['name']} (Pin {node.motors[node.current_idx]['pin']}) ---")
            elif ch == 'f': 
                node.motors[node.current_idx]['start_pwm'] = node.current_pwm
                node.current_idx = (node.current_idx + 1) % len(node.motors)
                node.current_pwm = node.motors[node.current_idx]['start_pwm']
                print(f"\n--- {node.motors[node.current_idx]['name']} (Pin {node.motors[node.current_idx]['pin']}) ---")
            
            node.current_pwm = max(100, min(600, node.current_pwm))
            node.send_cmd()
            deg = node.pwm_to_deg(node.current_pwm)
            name = node.motors[node.current_idx]['name']
            pin = node.motors[node.current_idx]['pin']
            print(f'\r[{node.last_status}] {name} (Pin {pin}) @ {node.current_pwm} ({deg:.1f}°)   ', end='', flush=True)

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        print('\n\nExited. Logs saved to /workspace/fr_leg_calibration.log.')
        if 'node' in locals():
            # Wait for thread to finish to avoid terminate exception
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
