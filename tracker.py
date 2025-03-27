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
        sum_right = float(sum(right))
        sum_left = float(sum(left))


        if centerAhead < 5.0: 
            avg_left = sum_left/ len(left)
            avg_right = sum_right/ len(right)

            error = avg_left - avg_right

            Kp = 1.5
            vel.angular.z = -Kp * error   # steer to balance distances
            self.publisher.publish(vel)

        else:
            vel.linear.x = 2.0 
        
            self.publisher.publish(vel)

        




def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
