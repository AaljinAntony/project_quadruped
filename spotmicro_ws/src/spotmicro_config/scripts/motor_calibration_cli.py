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
        # FINAL PCA mapping based on user request
        self.motor_names = [
            'FL Foot (Pin 0)', 'FL Leg (Pin 1)', 'FL Shoulder (Pin 2)',
            'RL Foot (Pin 4)', 'RL Leg (Pin 5)', 'RL Shoulder (Pin 6)',
            'FR Foot (Pin 8)', 'FR Leg (Pin 9)', 'FR Shoulder (Pin 10)',
            'RR Foot (Pin 12)', 'RR Leg (Pin 13)', 'RR Shoulder (Pin 14)'
        ]
        self.last_status = 'Waiting for ESP32...'
        self.log_file = '/workspace/calibration_results.txt'

    def status_callback(self, msg):
        self.last_status = msg.data

    def pwm_to_deg(self, pwm):
        # Calculation based on main.cpp.normal math:
        # pulse_us = 1500.0 + (radians * (1000.0 / 1.5708))
        # PWM = pulse_us / 4.8828
        # => degrees = (PWM * 4.8828 - 1500) * 0.09
        return (pwm * 4.8828 - 1500) * 0.09

    def send_cmd(self, log=True):
        msg = Int32MultiArray()
        msg.data = [int(self.current_motor), int(self.current_pwm)]
        self.publisher_.publish(msg)
        if log:
            deg = self.pwm_to_deg(self.current_pwm)
            with open(self.log_file, 'a') as f:
                f.write(f'[{datetime.datetime.now()}] MOTOR: {self.motor_names[self.current_motor]} (Idx {self.current_motor}) -> PWM: {self.current_pwm} (Approx {deg:.2f} deg)\n')

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
        node = MotorCalibrationClient()
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        print('\nSpotMicro Motor Calibration - FINAL MAPPING')
        print('A/F: Switch Motor | W/S: +/- 1 | E/D: +/- 10 | Q/Ctrl+C: Quit')
        print('----------------------------------------------------------')

        while True:
            ch = getch().lower()
            if ch == 'q' or ch == '\x03': # \x03 is Ctrl+C in raw mode
                break
            
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
            deg = node.pwm_to_deg(node.current_pwm)
            print(f'\r[{node.last_status}] {node.motor_names[node.current_motor]} @ {node.current_pwm} ({deg:.1f}°)   ', end='', flush=True)

    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f'\nError occurred: {e}')
    finally:
        print('\n\nExiting calibration. Results saved to /workspace/calibration_results.txt')
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
