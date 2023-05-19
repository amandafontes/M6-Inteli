#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose # Classe que contém o tipo da mensagem (x, y, z)

class TurtleController(Node): # Classe que herda de Node

    # Criando o construtor da classe superior
    def __init__(self):
        super().__init__('turtle_controller')
        # Criando a subscrição, utilizando um método de nós
        self.subscription = self.create_subscription(
            msg_type=Pose, # Tipo da mensagem
            topic='/turtle1/pose', # O número pode mudar de acordo com a tartaruga que temos na tela
            callback=self.pose_callback, # Método que será chamado toda vez que recebermos uma mensagem no tópico
            qos_profile=10, # Pilha de mensagens que será acumulada no histórico
        )

    def pose_callback(self, msg): # Método que será chamado toda vez que recebermos uma mensagem no tópico
        self.get_logger().info(f"A tartaruga está em x={msg.x}, y={msg.y}, theta={msg.theta}") # Serve para exibir na tela o que está acontecendo

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc) # Fica rodando até que o programa seja encerrado
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()