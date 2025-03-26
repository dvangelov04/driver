import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan 



# topics 

# publisher topics
steering = '/cmd_vel'

#subscriber topics
lidar = '/scan'


class Driver(Node): 
    def __init__(self):
        super().__init__('driverless')
        self.publisher = self.create_publisher(Twist, steering, 100)
        self.subscribe = self.create_subscription(LaserScan, lidar, self.velpub, 100)

        self.counter = 0 
        


    def velpub(self, msg):
        vel = Twist()
        
        centerAhead = msg.ranges[0]
        left = [r for r in msg.ranges[-180:] if math.isfinite(r)]
        right = [r for r in msg.ranges[:180] if math.isfinite(r)]
        sum_right = float(sum(right), 5)
        sum_left = float(sum(left), 5)

        if centerAhead <= 5:
            if sum_left > sum_right:
                vel.angular.z = 3.0
            elif sum_right > sum_left:
                vel.angular.z = -3.0
            else: 
                vel.linear.x = 0.0
                self.publisher.publish(vel)
        else:
        
            vel.linear.x = 3.0 
            self.publisher.publish(vel)
        




def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
