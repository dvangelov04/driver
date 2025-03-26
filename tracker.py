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
        leftAhead = msg.ranges[-30:0]
        rightAhead = msg.ranges[0:30]

        
        leftCleaned = [l for l in leftAhead if math.isinf(l)]
        rightCleaned = [r for r in rightAhead if math.isinf(r)]
        if centerAhead <= 5:
            
            if min(leftCleaned) <= 5.0:
                vel.angular.z = -3.0 
            elif min(rightCleaned) <= 5.0: 
                vel.angular.z = 3.0 
            else: 
                vel.linear.x = 0.0
            self.publisher.publish(vel)
        
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
