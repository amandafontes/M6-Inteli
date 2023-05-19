from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point
from tf_transformations import euler_from_quaternion
from time import sleep
import math
from fila import Fila

class TrutleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.publishers_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.msg = Twist()
        self.lista = Fila()
        
    def move_turtle(self):
        self.lista.tamanho()
        self.msg.linear.x, self.msg.angular.z = self.lista.chamada()
        self.msg._angular.z = math.radians(self.msg._angular.z)
        self.publishers_.publish(self.msg)
        sleep(1.0)
        if self.lista.status() == 0:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.publishers_.publish(self.msg)
            sleep(1.0)
            exit()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TrutleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()