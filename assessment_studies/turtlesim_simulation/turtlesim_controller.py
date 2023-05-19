#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TPose # Pose do turtlesim que iremos criar
from geometry_msgs.msg import Twist

max_difference = 0.1

class Pose(TPose):

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)

    def __repr__(self):
        return f"(x={self.x}, y={self.y})" # Assim, podemos printar uma pose e ela já será exibida. Não precisamos criar esse código de novo, toda vez, dentro do print

    def __add__(self, other): # Para dizer que a adição de duas poses nada mais é do que adição dos atributos dessa pose
        # Essa função fala para o Python executar esse método, de adição, toda vez que ele ver um "+"
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other): # Para dizer que a adição de duas poses nada mais é do que adição dos atributos dessa pose
        # Essa função fala para o Python executar esse método, de subtração, toda vez que ele ver um "-"
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other): # Para dizer que a adição de duas poses nada mais é do que adição dos atributos dessa pose
        # Essa função fala para o Python fazer overload do operador "=="
        return abs(self.x - other.x) < max_difference \
        and abs(self.y - other.y) < max_difference \
        # max_difference é a nossa margem de erro. Se a diferença entre os valores for menor que a margem de erro, então consideramos que os valores são iguais
    
class TurtleController(Node): # Classe que herda de Node. É um nó do ROS

    # Criando o construtor da classe superior
    def __init__(self, control_period=0.02): # O período de controle é o tempo que o programa vai esperar para executar o callback novamente. No caso, 2 milissegundos
        super().__init__('turtle_controller')
        self.pose = Pose(x=-40.0)
        self.setpoint = Pose(x=-40.0) # É um valor relativo à pose atual, para facilitar o controle do sistema
        self.publisher = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )
        # Criando a subscrição, utilizando um método de nós
        self.subscription = self.create_subscription(
            msg_type=Pose, # Tipo da mensagem
            topic='/turtle1/pose', # O número pode mudar de acordo com a tartaruga que temos na tela
            callback=self.pose_callback, # Método que será chamado toda vez que recebermos uma mensagem no tópico
            qos_profile=10 # Pilha de mensagens que será acumulada no histórico
        )
        # Criando o nosso clock para haver controle sobre a periodicidade do callback
        self.control_timer = self.create_timer(
            timer_period_sec=control_period,
            callback=self.control_callback
        )

    # Definindo o loop de controle:

    def control_callback(self):
        # Verificar se já existe a informação de Pose
        if self.pose.x == -40.0:
            self.get_logger().info("Aguardando primeira pose...")
            return # É uma cláusula de guarda a partir da qual iremos construir o callback de controle
        
        msg = Twist()

        x_difference = self.setpoint.x - self.pose.x
        y_difference = self.setpoint.y - self.pose.y

        # Verificar se a tartaruga já chegou no setpoint
        if self.pose == self.setpoint:
            msg.linear.x, msg.linear.y = 0.0, 0.0
            self.get_logger().info(f"Chegou no setpoint: {self.setpoint}")
            exit()

        if abs(y_difference) > max_difference:
            msg.linear.y = 1.0 if y_difference > 0 else -1.0
        else:
            msg.linear.y = 0.0

        if abs(x_difference) > max_difference:
            msg.linear.x = 1.0 if x_difference > 0 else -1.0
        else:
            msg.linear.x = 0.0

        self.publisher.publish(msg)

    def pose_callback(self, msg): # Método que será chamado toda vez que recebermos uma mensagem no tópico
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == -40.0: # Na primeira vez que realiza o callback, percebe-se que ainda não foi definido o setpoint
            self.setpoint = self.pose + Pose(x=1.0, y=1.0) # Cria o setpoint pela pose que acabou de pegar, mais x=1.0. O símbolo de adição chama o método add da classe Pose
        self.get_logger().info(f"A tartaruga está em x={msg.x}, y={msg.y}, theta={msg.theta}") # Serve para exibir na tela o que está acontecendo

def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc) # Fica rodando até que o programa seja encerrado
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()