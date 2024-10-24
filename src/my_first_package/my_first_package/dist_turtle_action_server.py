import time
import math
import rclpy as rp

from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_first_package_msgs.action import DistTurtle
from my_first_package.my_subscriber import TurtlesimSubscriber


class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0.
        self.is_first_time = True
        self.current_pose = Pose()
        self.prev_pose = Pose()

        self.publisher = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10
        )

        self.action_server = ActionServer(
            self, 
            DistTurtle, 
            'dist_turtle', 
            self.execute_callback
        )

        self.declare_parameter('quantile_time', 0.75)
        self.declare_parameter('almost_goal_time', 0.95)
        (quantile_time, almost_goal_time) = self.get_parameters([
            'quantile_time', 
            'almost_goal_time'
            ])
        self.quantile_time = quantile_time.value
        self.almost_goal_time = almost_goal_time.value
        
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            print(f'{param.name} is changed to {param.value}')
            if param.name == 'quantile_time':
                self.quantile_time = param.value
            elif param.name == 'almost_goal_time':
                self.almost_goal_time = param.value
        
        print(f'Now qunatile_time: {self.quantile_time}, almost_goal_time: {self.almost_goal_time}')
        return SetParametersResult(successful=True)

    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback()

        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remained_dist = goal_handle.request.dist-self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.publisher.publish(msg)
            time.sleep(0.01)

            if feedback_msg.remained_dist<0.2:
                break
        
        goal_handle.succeed()
        result = DistTurtle.Result()

        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = self.total_dist

        self.total_dist = 0
        self.is_first_time = True
        return result
    
    def calc_diff_pose(self):
        if self.is_first_time:
            self.prev_pose = self.current_pose
            self.is_first_time = False

        diff_dist = math.sqrt((self.current_pose.x-self.prev_pose.x)**2+(self.current_pose.y-self.prev_pose.y)**2)
        self.prev_pose = self.current_pose
        return diff_dist


class TurtleSub_Action(TurtlesimSubscriber):
    def __init__(self, action_server):
        super().__init__()
        self.action_server = action_server

    def callback(self, msg):
        self.action_server.current_pose = msg

def main(args=None):
    rp.init(args=args)

    executor = MultiThreadedExecutor()

    dist_turtle_action_server = DistTurtleServer()
    subscriber = TurtleSub_Action(action_server = dist_turtle_action_server)

    executor.add_node(dist_turtle_action_server)
    executor.add_node(subscriber)

    try:
        executor.spin()

    finally:
        executor.shutdown()
        dist_turtle_action_server.destroy_node()
        subscriber.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()