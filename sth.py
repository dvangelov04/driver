import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.subscribe = self.create_subsciber(LaserScan, lidar, self.velpub 100)

        self.counter = 0 
        


    def velpub(self, msg):
        vel = Twist()
        
        distanceAhead = msg.ranges[0]
        distancesLeft = msg.ranges[-45:]
        distancesRight = msg.ranges[:45]

        if distanceAhead < 10: 
            if float('inf') in distancesRight: 
                vel.angular.z = -1.0
            if float('inf') in distancesLeft: 
                vel.angular.z = 1.0
            self.counter += 1
            self.publisher.publish (vel)
                
        else: 
            
            vel.linear.x = 1.0 
            self.counter += 1
            self.publisher.publish(vel)


        if self.counter == 30: 
            vel = Twist()
            self.publisher.publish(vel)






def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()
