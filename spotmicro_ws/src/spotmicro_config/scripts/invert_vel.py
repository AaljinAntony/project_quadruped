import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityInverter(Node):
    def __init__(self):
        super().__init__('velocity_inverter')
        # Listen to the PRIMARY standard teleop input
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        # Publish to the newly namespaced topic
        self.publisher_ = self.create_publisher(Twist, '/robotA/cmd_vel', 10)

    def listener_callback(self, msg):
        inverted = Twist()
        inverted.linear.x = -msg.linear.x
        inverted.linear.y = -msg.linear.y
        inverted.angular.z = msg.angular.z 
        self.publisher_.publish(inverted)

def main():
    rclpy.init()
    node = VelocityInverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
