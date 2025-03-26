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

        obstacle_threshold = 5.0

        if centerAhead <= obstacle_threshold:
            left_blocked = any(r <= obstacle_threshold for r in leftAhead)
            right_blocked = any(r <= obstacle_threshold for r in rightAhead)

            vel.linear.x = 0.0  # Stop forward motion when deciding where to turn

            if not right_blocked:
                vel.angular.z = -3.0  # turn right
                self.get_logger().info("Turning right")
            elif not left_blocked:
                vel.angular.z = 3.0   # turn left
                self.get_logger().info("Turning left")
            else:
                vel.angular.z = 0.0   # blocked both sides
                self.get_logger().info("Blocked on all sides, staying still.")
        else:
            vel.linear.x = 3.0
            vel.angular.z = 0.0
            self.get_logger().info("Path clear. Moving forward.")

        self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()