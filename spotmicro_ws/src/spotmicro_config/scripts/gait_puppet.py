import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class GaitPuppet(Node):
    def __init__(self):
        super().__init__('gait_puppet')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.start_time = time.time()
        
        # Joint Names
        self.names = [
            'motor_front_left_shoulder', 'motor_front_left_leg', 'foot_motor_front_left',
            'motor_rear_left_shoulder', 'motor_rear_left_leg', 'foot_motor_rear_left',
            'motor_front_right_shoulder', 'motor_front_right_leg', 'foot_motor_front_right',
            'motor_rear_right_shoulder', 'motor_rear_right_leg', 'foot_motor_rear_right'
        ]
        self.get_logger().info('Gait Puppet Started: Manual Trot Verification.')

    def timer_callback(self):
        t = time.time() - self.start_time
        freq = 0.5 # 2 seconds per cycle
        
        # Trot phase (0 to 1)
        phase = (t * freq) % 1.0
        
        # Simple sinusoidal leg lift
        lift_amp = 0.3
        
        # Diagonal Pairs: (FL, RR) and (FR, RL)
        pos = [0.0] * 12
        
        # Pair A (FL, RR) lift during phase 0.0 - 0.5
        if phase < 0.5:
            lift = math.sin(phase * math.pi * 2) * lift_amp
            # FL Leg (Index 1) and RR Leg (Index 10)
            pos[1] = lift 
            pos[10] = lift
            # FL Foot (Index 2) and RR Foot (Index 11) - slight tuck
            pos[2] = -lift * 0.5
            pos[11] = -lift * 0.5
        else:
            # Pair B (FR, RL) lift during phase 0.5 - 1.0
            lift = math.sin((phase - 0.5) * math.pi * 2) * lift_amp
            # FR Leg (Index 7) and RL Leg (Index 4)
            pos[7] = lift
            pos[4] = lift
            # FR Foot (Index 8) and RL Foot (Index 5)
            pos[8] = -lift * 0.5
            pos[5] = -lift * 0.5

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = pos
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitPuppet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
