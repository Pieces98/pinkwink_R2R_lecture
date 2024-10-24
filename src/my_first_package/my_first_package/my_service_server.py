import time
import numpy as np
import rclpy as rp

from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute
from my_first_package_msgs.srv import MultiSpawn


class MultiSpawning(Node):
    def __init__(self):
        super().__init__('multi_spawn')
        self.server = self.create_service(
            MultiSpawn, 
            'multi_spawn', 
            self.callback_service
        )

        self.teleport = self.create_client(
            TeleportAbsolute, 
            '/turtle1/teleport_absolute'
        )

        self.spawn = self.create_client(
            Spawn, 
            '/spawn'
        )

        self.req_teleport = TeleportAbsolute.Request()
        self.req_spawn = Spawn.Request()
        self.x0 = 5.54
        self.y0 = 5.54

    def calc_circular_position(self, n, r):
        theta = np.arange(0, 2*np.pi, 2*np.pi/n)
        x = self.x0+r*np.cos(theta)
        y = self.y0+r*np.sin(theta)
        return x, y, theta

    def callback_service(self, request, response):
        x, y, theta = self.calc_circular_position(request.num, 3)
        for x_i, y_i, theta_i in zip(x, y, theta):
            self.req_spawn.x = x_i
            self.req_spawn.y = y_i
            self.req_spawn.theta = theta_i
            self.spawn.call_async(self.req_spawn)
            time.sleep(0.1)

        response.x = x.tolist()
        response.y = y.tolist()
        response.theta = theta.tolist()
        return response
    
def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)

    multi_spawn.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
