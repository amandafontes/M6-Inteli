#! /usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from collections import deque
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose

margem_erro = 0.1

# Construtor da classe MissionControl
class MissionControl(deque):
    
    def __init__(self, csv_file="pontos.csv"):
        super().__init__()

        # Fazendo a leitura do arquivo csv, que contém os pontos relativos à trajetória da tartaruga
        with open(csv_file) as csv_arquivo:
            csv_reader = csv.reader(csv_arquivo, delimiter=',')

            for row in csv_reader:
                new_pose = Pose() # Para gerar uma fila cuja estrutura de dados se relaciona ao método Pose
                new_pose.x, new_pose.y = [float(x) for x in row] # Transformando em um valor float para cada item da linha
                self.enqueue(new_pose)

    def enqueue(self, x): # Para acrescentar à fila
        super().append(x)

    def dequeue(self): # Para remover da fila
        return super().popleft()

    def pop(self): # Para remover o último elemento na pilha (útil para fazer a tartaruga retornar ao ponto inicial)
        return super().pop()

# Classe que contém os métodos referentes à manipulação dos movimentos da tartaruga
class Pose(TPose):

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)

    def __repr__(self):
        return f"(x={self.x}, y={self.y})" # Para exibir as coordenadas de maneira contínua

    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        return abs(self.x - other.x) < margem_erro \
        and abs(self.y - other.y) < margem_erro \
        # margem_erro é a nossa margem de erro. Se a diferença entre os valores for menor que a margem de erro, então consideramos que os valores são iguais
    
class TurtleController(Node): # Classe que herda de Node. É um nó do ROS.

    # Criando o construtor da classe superior
    def __init__(self, mission_control, control_period=0.02): # O período de controle é o tempo que o programa vai esperar para executar o callback novamente. No caso, 2 milissegundos
        super().__init__('turtle_controller')
        self.pose = Pose(x=0.0)
        self.setpoint = Pose(x=0.0) # É um valor relativo à pose atual, para facilitar o controle do sistema
        self.mission_control = mission_control
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )

        # Criando a subscrição
        self.subscription = self.create_subscription(
            msg_type=Pose, # Tipo da mensagem
            topic='/turtle1/pose', # Tópico que representa a posição da tartaruga
            callback=self.subscriber_callback, # Método que será chamado para receber as mensagens publicadas no tópico
            qos_profile=10 # Pilha de mensagens que será acumulada no histórico
        )

        # Criando o clock para haver controle sobre a periodicidade do callback
        self.control_timer = self.create_timer(
            timer_period_sec=control_period,
            callback=self.publisher_callback
        )

    # Definindo o loop de controle:

    def publisher_callback(self):

        # Verificar se já existem informações publicadas de Pose
        if self.pose.x == 0.0:
            self.get_logger().info("Aguardando primeira pose...")
            return
        
        msg = Twist()

        x_difference = self.setpoint.x - self.pose.x
        y_difference = self.setpoint.y - self.pose.y

        # Verificando se a tartaruga já se encontra no setpoint
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.update_setpoint()

        if abs(y_difference) > margem_erro:
            msg.linear.y = 1.0 if y_difference > 0 else -1.0
        else:
            msg.linear.y = 0.0

        if abs(x_difference) > margem_erro:
            msg.linear.x = 1.0 if x_difference > 0 else -1.0
        else:
            msg.linear.x = 0.0

        self.publisher.publish(msg)
    
    def update_setpoint(self): # Para atualizar o setpoint da tartaruga
        try:
            self.setpoint = self.pose + self.mission_control.dequeue() # Pega o primeiro valor da fila
            self.get_logger().info(f"A tartaruga chegou em {self.pose} e agora vai para {self.setpoint}")
        except IndexError:
            self.get_logger().info(f"A tartaruga finalizou sua trajetória.")
            exit()

    def subscriber_callback(self, msg): # Método chamado quando uma mensagem é recebida no tópico
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == 0.0: # Na primeira vez que se realiza o callback, percebe-se que ainda não foi definido o setpoint
            self.update_setpoint()
        self.get_logger().info(f"A tartaruga está em x={msg.x}, y={msg.y}, theta={msg.theta}")

def main(args=None):
    rclpy.init(args=args)
    mc = MissionControl()
    tc = TurtleController(mc)
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()