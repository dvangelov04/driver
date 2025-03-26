import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

# topics 
steering = '/cmd_vel'
lidar = '/scan'

class Driver(Node): 
    def __init__(self):
        super().__init__('driverless')
        self.publisher = self.create_publisher(Twist, steering, 100)
        self.subscription = self.create_subscription(LaserScan, lidar, self.velpub, 100)

    def velpub(self, msg):
        vel = Twist()
        
        centerAhead = msg.ranges[0] if math.isfinite(msg.ranges[0]) else float('inf')
        leftAhead = [r for r in msg.ranges[-30:] if math.isfinite(r)]
        rightAhead = [r for r in msg.ranges[:30] if math.isfinite(r)]

        # Check for obstacles
        left_blocked = any(r < 5.0 for r in leftAhead)
        right_blocked = any(r < 5.0 for r in rightAhead)

# Decision logic
        if left_blocked and not right_blocked:
            vel.angular.z = -3.0  # Turn right
            vel.linear.x = 0.0
            self.get_logger().info("Obstacle on the left — turning right.")
        elif right_blocked and not left_blocked:
            vel.angular.z = 3.0   # Turn left
            vel.linear.x = 0.0
            self.get_logger().info("Obstacle on the right — turning left.")
        elif right_blocked and left_blocked:
            vel.angular.z = 0.0   # No clear path
            vel.linear.x = 0.0
            self.get_logger().info("Blocked on both sides — stopping.")
        else:
            vel.linear.x = 3.0
            vel.angular.z = 0.0
            self.get_logger().info("Path is clear — moving forward.")

        self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()