#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import sys
import termios
import tty
import threading
import datetime

class MotorCalibrationClient(Node):
    def __init__(self):
        super().__init__('motor_calibration_client')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/motor_calibration', 10)
        self.subscription = self.create_subscription(String, '/motor_status', self.status_callback, 10)
        self.current_motor = 0
        self.current_pwm = 300
        # Corrected Mapping based on your manual feedback
        self.motor_names = [
            'FL Foot (P0)',   'FL Leg (P1)',    'FL Shoulder (P2)',
            'BL Leg (P4)',    'BL Shoulder (P5)', 'FR Foot (P6)',
            'FR Shoulder (P8)', 'BR Foot (P9)',   'BR Leg (P10)',
            'BR Shoulder (P12)', 'BL Foot (P13)', 'FR Leg (P14)'
        ]
        self.last_status = 'Waiting for ESP32...'
        self.log_file = '/workspace/calibration_results.txt'

    def status_callback(self, msg):
        self.last_status = msg.data

    def send_cmd(self, log=True):
        msg = Int32MultiArray()
        msg.data = [int(self.current_motor), int(self.current_pwm)]
        self.publisher_.publish(msg)
        if log:
            with open(self.log_file, 'a') as f:
                f.write(f'MOTOR: {self.motor_names[self.current_motor]} -> PWM: {self.current_pwm}\n')

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
    rclpy.init()
    node = MotorCalibrationClient()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print('SpotMicro Motor Calibration - CORRECTED MAPPING')
    print('A/F: Switch Motor | W/S: +/- 1 | E/D: +/- 10 | Q: Quit')

    try:
        while True:
            ch = getch().lower()
            if ch == 'q': break
            
            if ch == 'w': node.current_pwm += 1
            elif ch == 's': node.current_pwm -= 1
            elif ch == 'e': node.current_pwm += 10
            elif ch == 'd': node.current_pwm -= 10
            elif ch == 'a': 
                node.current_motor = (node.current_motor - 1) % 12
                node.current_pwm = 300
                print(f'\n--- {node.motor_names[node.current_motor]} ---')
            elif ch == 'f': 
                node.current_motor = (node.current_motor + 1) % 12
                node.current_pwm = 300
                print(f'\n--- {node.motor_names[node.current_motor]} ---')
            
            node.current_pwm = max(100, min(600, node.current_pwm))
            node.send_cmd()
            print(f'\r[{node.last_status}] {node.motor_names[node.current_motor]} @ {node.current_pwm}   ', end='', flush=True)
    except Exception as e:
        print(f'\nError: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
