import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
import math
from pilha import Pilha

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.publishers_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.msg = Twist()
        self.pilha = Pilha()

    def move_turtle(self):
        self.pilha.status()
        self.msg.linear.x, self.msg.angular.z = self.pilha.desempilhar()
        self.msg.angular.z = math.radians(self.msg.angular.z)
        self.publishers_.publish(self.msg)
        sleep(1.0)
        if self.pilha.tamanho() == 0:
            print("A Pilha está vazia")
            exit()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()