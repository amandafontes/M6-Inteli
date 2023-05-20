#! /usr/bin/env python3

# Importação dos módulos e bibliotecas necessários

import csv
import math
import rclpy

from math import atan2
from rclpy.node import Node
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf_transformations import euler_from_quaternion

margem = 0.1 # Margem de erro relativa à comparação entre a posição atual do robô e a próxima posição da fila

pontos = [(0.0, 0.0),
         (1.0, 1.0),
         (2.0, -1.0),
         (3.0, -1.0),
         (5.0, 2.0),
         (5.0, 3.0),]

# Construção da classe responsável pelo controle de movimentação do robô

class TurtleController(Node):

    # Função que cria os parâmetros referentes à movimentação do robô, o publisher, o subscriber e o timer
    def __init__(self, pontos, control_period=0.05):

        # Instancia as variáveis relativas à movimentação e pontos da trajetória
        super().__init__('subscriber_node')
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.point = 0
        self.point_list = pontos

        # Cria o publisher, que envia comandos de velocidade via tópico /cmd_vel
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10)
        
        # Cria o subscriber, que recebe informações de odometria do robô via tópico /odom
        self.subscription = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.subscriber_callback,
            qos_profile=4)

        # Define o controle de tempo para 2 milissegundos 
        self.timer = self.create_timer(
            timer_period_sec=0.02,
            callback=self.publisher_callback)


    # Função responsável pela execução correta da trajetória do robô, considerando rotação e movimentação linear
    def publisher_callback(self):

        goal = Point() # Apontar para o próximo ponto da trajetória
        goal.x = self.point_list[self.point][0]
        goal.y = self.point_list[self.point][1]
        
        # Obtenção da inclinação necessária para apontar para o próximo ponto da trajetória
        inc_x = goal.x - self.x
        inc_y = goal.y - self.y
        angulo_rotacao = atan2(inc_y,inc_x) # Cálculo do ângulo de rotação necessário para apontar para o próximo ponto da trajetória
        
        velocidade = Twist()
        
        # Atualiza o próximo ponto da trajetória caso a inclinação desejada seja inferior à margem definida
        if (abs(inc_x) < margem and abs(inc_y) < margem):
            self.point = 0 if (len(self.point_list) == self.point + 1) else (self.point + 1)
        
        # Executa a rotação para o próximo ponto da trajetória, caso a inclinação seja superior à margem definida
        if abs(angulo_rotacao - self.theta) > margem:
            velocidade.linear.x = 0.0 # Velocidade linear se mantém nula para que a rotação seja executada
            velocidade.angular.z = 0.3 if (angulo_rotacao - self.theta) > 0.0 else -0.3
        else: # Executa a velocidade linear caso a inclinação desejada corresponda à margem deginida
            velocidade.linear.x = 0.3
            velocidade.angular.z = 0.0
        self.publisher.publish(velocidade) # Publica o comando de velocidade para a locomoção do robô

    # Função que envia as informações para o subscriber
    def subscriber_callback(self, msg):

        # Obtenção das coordenadas via atributos x e y do tópico de odometria
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Obtenção da rotação realizada via atributo de orientação da odometria
        rot = msg.pose.pose.orientation
        _,_,self.theta = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])

        # Obtenção da posição do robô
        self.get_logger().info(f"x={self.x:3f}, y={self.y:3f}")

# Construção da função main, responsável pela manutenção da execução das classes construídas

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController(pontos) # TurtleController recebe "pontos" como parâmetro, ou seja, a rota a ser percorrida pelo robô
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()