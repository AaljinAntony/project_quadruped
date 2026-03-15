import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import sys
import termios
import tty
import threading

class MotorCalibrationClient(Node):
    def __init__(self):
        super().__init__('motor_calibration_client')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/motor_calibration', 10)
        self.subscription = self.create_subscription(String, '/motor_status', self.status_callback, 10)
        self.current_motor = 0
        self.current_pwm = 307 # ~1500us
        self.motor_names = [
            'FL Shoulder', 'FL Leg', 'FL Foot',
            'FR Shoulder', 'FR Leg', 'FR Foot',
            'RL Shoulder', 'RL Leg', 'RL Foot',
            'RR Shoulder', 'RR Leg', 'RR Foot'
        ]
        self.last_status = 'Waiting for ESP32...'

    def status_callback(self, msg):
        self.last_status = msg.data

    def send_cmd(self):
        msg = Int32MultiArray()
        msg.data = [int(self.current_motor), int(self.current_pwm)]
        self.publisher_.publish(msg)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b': # Handle arrows
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

    print('--- SpotMicro Motor Calibration CLI ---')
    print('Controls: W/S: +/- 1 Tick | E/D: +/- 10 Ticks')
    print('          A/F: Prev/Next Motor | Q: Quit')
    print('---------------------------------------')

    try:
        while True:
            ch = getch().lower()
            if ch == 'q': break
            elif ch == 'w': node.current_pwm += 1
            elif ch == 's': node.current_pwm -= 1
            elif ch == 'e': node.current_pwm += 10
            elif ch == 'd': node.current_pwm -= 10
            elif ch == 'a': node.current_motor = (node.current_motor - 1) % 12
            elif ch == 'f': node.current_motor = (node.current_motor + 1) % 12
            
            # Constrain PWM (PCA9685 ticks, roughly 100 to 500)
            node.current_pwm = max(100, min(500, node.current_pwm))
            
            node.send_cmd()
            print(f'\r[{node.last_status}] | Target: {node.motor_names[node.current_motor]} (P {node.current_motor}) @ {node.current_pwm}   ', end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print('\nExiting...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
