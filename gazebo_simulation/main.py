#! /usr/bin/env python3

# Importação dos módulos e bibliotecas necessários

import csv
import math
import rclpy

from rclpy.node import Node
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist as Twist
from tf_transformations import euler_from_quaternion

max_difference = 0.1 # Margem de erro relativa à comparação entre a posição atual do robô e a próxima posição da fila

# Construção da classe responsável por gerar a fila de posições pelas quais o robô irá passar

class MissionControl(deque):

    def __init__(self, csv_file="turtlebot_route.csv"):
        super().__init__()

        # Leitura do arquivo csv que contém os pontos
        with open(csv_file) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')

            # Transformação dos pontos da rota em valores float para que seja construída a fila de posições
            for row in csv_reader:
                new_pose = Twist()
                new_pose.x, new_pose.y = [float(x) for x in row]
                self.enqueue(new_pose)

    # Função para enfileirar as posições extraídas do arquivo csv
    def enqueue(self, x):
        super().append(x)

    # Função para remover da fila as posições que já foram percorridas
    def dequeue(self):
        return super().popleft()

# Construção da classe que contém os métodos relativos à sequência de posições do robô

class Twist(Twist):

        def __init__(self, x=0.0, y=0.0, z=0.0):
            super().__init__(x=x, y=y)
    
        # Para retornar diretamente a posição
        def __repr__(self):
            return f"(x={self.x}, y={self.y})"
    
        # Para adicionar à posição quando identificado o operador +
        def __add__(self, other):
            self.x += other.x
            self.y += other.y
            return self
    
        # Para subtrair da posição quando identificado o operador -
        def __sub__(self, other):
            self.x -= other.x
            self.y -= other.y
            return self
        
        # Para realizar o overload da posição quando identificado o operador de igualdade
        def __eq__(self, other):
            return abs(self.x - other.x) < max_difference \
            and abs(self.y - other.y) < max_difference \
            # Se a diferença entre a posição atual e a próxima da fila for menor que a margem de erro, consideramos que as posições são iguais

# Construção da classe responsável pelo controle de movimentação do robô

class TurtleController(Node):

    def __init__(self, mission_control, control_period=0.05):
        super().__init__('turtle_controller')
        self.pose = Twist(x=-40.0)
        self.setpoint = Twist(x=-40.0)
        self.mission_control = mission_control

        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10
        )
        
        self.subscription = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.pose_callback,
            qos_profile=10
        )

        self.control_timer = self.create_timer(
            timer_period_sec=control_period,
            callback=self.control_callback
        )
    
    def control_callback(self):

        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        
        msg = Twist()

        x_difference = self.setpoint.x - self.pose.x
        y_difference = self.setpoint.y - self.pose.y

        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()

        if abs(y_difference) > max_difference:
            msg.linear.y = 1.0 if y_difference > 0 else -1.0
        else:
            msg.linear.y = 0.0

        if abs(x_difference) > max_difference:
            msg.linear.x = 1.0 if x_difference > 0 else -1.0
        else:
            msg.linear.x = 0.0

        self.publisher.publish(msg)

    def update_setpoint(self):
        try:
            self.setpoint = self.pose + self.mission_control.dequeue() # Dequeue pega o primeiro valor da fila
            self.get_logger().info(f"O robô chegou em {self.pose} e agora vai para {self.setpoint}")
        except IndexError:
            self.get_logger().info(f"A rota foi finalizada.")
            exit()

    def pose_callback(self, msg):
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == -40.0: 
            self.update_setpoint()
        self.get_logger().info(f"O robô está em x={msg.x}, y={msg.y}, theta={msg.theta}")

# Construção da função main, responsável pela manutenção da execução das classes construídas

def main(args=None):
    rclpy.init(args=args)
    mc = MissionControl()
    tc = TurtleController(mc)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()