import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SpotMicroPuppet(Node):
    def __init__(self):
        super().__init__('spotmicro_puppet')
        # We publish directly to the topic your ESP32 is listening to
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(4.0, self.timer_callback) # Triggers every 4 seconds for safety
        self.is_standing = False
        self.get_logger().info('SpotMicro Puppet Node Started. Calibrated for real hardware.')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Exact joint names used in CHAMP and mapped in ESP32 firmware
        msg.name = [
            'motor_front_left_shoulder', 'motor_front_left_leg', 'foot_motor_front_left',
            'motor_front_right_shoulder', 'motor_front_right_leg', 'foot_motor_front_right',
            'motor_rear_left_shoulder', 'motor_rear_left_leg', 'foot_motor_rear_left',
            'motor_rear_right_shoulder', 'motor_rear_right_leg', 'foot_motor_rear_right'
        ]

        if self.is_standing:
            self.get_logger().info('Commanding: SIT (Tucked / Calibrated Safety)')
            # Conservative Sit pose: Slightly legs out and feet tucked
            # Values are in Radians, strictly within URDF limits
            sit_shoulder = 0.0
            sit_leg = 0.3      
            sit_foot = -0.4    
            
            msg.position = [
                sit_shoulder, sit_leg, sit_foot,
                sit_shoulder, sit_leg, sit_foot,
                sit_shoulder, sit_leg, sit_foot,
                sit_shoulder, sit_leg, sit_foot
            ]
        else:
            self.get_logger().info('Commanding: STAND (Neutral Zero)')
            # Stand: All servos at their calibrated center points (0.0 radians)
            msg.position = [0.0] * 12
            
        self.publisher_.publish(msg)
        self.is_standing = not self.is_standing

def main(args=None):
    rclpy.init(args=args)
    node = SpotMicroPuppet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Puppet Node Stopped.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
