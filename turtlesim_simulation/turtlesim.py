# A linha de código abaixo especifica qual interpretador será necessário para rodar o script

#!/usr/bin/env python3
import time
import rclpy # Biblioteca necessária para interface entre Python e ROS2
from rclpy.node import Node # Importa a classe referente ao nó na rede ROS
from geometry_msgs.msg import Twist # Importa as mensagens padrão referentes a dados geométricos no ROS

# Criação da classe correspondente ao controle de movimento da tartaruga
class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) # Cria um publisher para enviar Twist messages de controle do movimento da tartaruga
        self.timer_ = self.create_timer(0.1, self.move_turtle) # Cria um timer para chamar a função move_turtle repetidamente
        self.twist_msg_ = Twist() # Inicializa uma mensagem Twist

    # Função para definir a velocidade linear e a velocidade angular correspondentes à mensagem Twist
    def move_turtle(self):

        # Loop que define a quantidade de vezes que a tartaruga irá realizar o movimento linear e o movimento de rotação
        for _ in range(11): # Onze será o número necessário para completar a figura geométrica

            # Velocidade linear configurada para 2 para fazer a tartaruga se mover linearmente
            self.twist_msg_.linear.x = 2.0
            self.twist_msg_.angular.z = 0.0
            self.publisher_.publish(self.twist_msg_)
            time.sleep(1.0) # Intervalo de um segundo para o próximo movimento
            
            # Velocidade angular configurada para 2.84 para rotacioná-la em 45 graus
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 2.84
            self.publisher_.publish(self.twist_msg_)
            time.sleep(1.0) # Intervalo de um segundo para o próximo movimento
        
        # Para a tartaruga assim que a figura geométrica é finalizada, fazendo com que ambas as velocidades aproximem-se de zero
        self.twist_msg_.linear.x = 0.01
        self.twist_msg_.angular.z = 0.01
        self.publisher_.publish(self.twist_msg_)

        # Para o timer para evitar que a tartaruga continue se movimentando após completar a figura geométrica
        self.timer_.cancel()

# Criação da função principal, em que as funções necessárias são chamadas
def main(args=None):
    rclpy.init() # Inicialização do sistema ROS
    turtle_controller = TurtleController() # Criação de instância da classe TurtleController
    rclpy.spin(turtle_controller) # Manutenção do nó em execução de forma contínua para receber as mensagens do publisher
    turtle_controller.destroy_node() # Encerramento do nó quando o processo é finalizado
    rclpy.shutdown() # Encerramento do sistema ROS

if __name__ == '__main__': # Confere se o script está sendo executado enquanto programa principal
    main() # Chama a função principal para inicializar o nó e iniciar o controle da movimentação da tartaruga