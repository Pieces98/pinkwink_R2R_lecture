import rclpy as rp

from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimPublisher(Node):
    def __init__(self):
        super().__init__('turtlesim_publisher')
        self.publication = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publication.publish(msg)


def main(args=None):
    rp.init(args=args)

    turtlesim_publisher = TurtlesimPublisher()
    rp.spin(turtlesim_publisher)

    turtlesim_publisher.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
