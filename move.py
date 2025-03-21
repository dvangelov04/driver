import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

import time


# which topic should the message be published on 
topicName = '/cmd_vel'


class VelPub(Node):
    def __init__(self):
        super.__init__('simple')
        self.publisherCreated = self.create_publisher(Twist, topicName, 100)
        self.counter = 0 
        communicationPeriod = 1 

        self.period = self.create_timer(communicationPeriod, self.PublisherCallBack)




    def PublisherCallBack(self):
        twist_msg = Twist()
        
        if self.counter == 20:
            twist_msg.linear.x = 0

        else: 
            self.counter += 1
            twist_msg.linear.x = 0.2
        self.publisherCreated.publish(twist_msg)

        # self.get_logger().info('Linear speed: ', "%s" twist_msg.linear.x)


        
def main(args=None):
    rclpy.init(args=args)
    publisherNode = publisherNode()

    rclpy.spin(publisherNode)

    publisherNode.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()