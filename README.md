<h2>Turtlesim: simulando um ambiente robótico integrado no ROS</h2>

<h3>Enunciado</h3>

<p>Crie um script em Python capaz de interagir com o nó de simulação do turtlesim e enviar mensagens nos tópicos que regem a locomoção da tartaruga principal. Utilize este script para reproduzir um desenho de sua autoria. Utilize a estrutura de dados que preferir para representar a “imagem” a ser desenhada. O uso de programação orientada a objetos é obrigatório.</p>

<h3>Demonstração</h3>

https://user-images.githubusercontent.com/77015911/234037806-c4661724-a9df-47ca-83de-ec9be10e01dd.mp4

<h3>Implementação</h3>

<p>Para a construção da atividade, será necessário, primeiramente, instalar e configurar o Windows Subsystem for Linux (WSL). Posteriormente, o mesmo procedimento deve ser feito para o Robot Operating System (ROS), sistema operacional que será utilizado para construir a simulação.</p>

<p>Uma vez configurado o ambiente de desenvolvimento, é necessário abrir o script Python, localizado no diretório <code>turtlesim_simulation/turtlesim.py</code>. O código é responsável por interagir com a aplicação correspondente ao Turtlesim, e irá fornecer as instruções necessárias para que a tartaruga se movimente a fim de completar a figura geométrica que queremos obter.</p>

<p>Sabendo-se que foi criado um ambiente virtual Python especialmente para a simulação, será necessário configurá-lo. Primeiramente, vá até o diretório <code>turtlesim_simulation</code>, abra o terminal, no WSL, e digite o seguinte comando:</p>

```powershell
pip install requirements.txt
```

<p>Agora que você instalou os módulos e pacotes utilizados no diretório, deve ativar o ambiente virtual Python criado. Isso pode ser feito por meio do seguinte comando:</p>

```powershell
source turtlesim_env/bin/activate
```

<p>O próximo passo é segmentar o terminal em duas partes. Em um terminal, abriremos o Turtlesim. No outro, iremos executar o script Python. No primeiro terminal, digite o seguinte comando:</p>

```powershell
python3 turtlesim.py
```

<p>No segundo terminal, digite o seguinte comando:</p>

```powershell
ros2 run turtlesim turtlesim_node
```

<p>Vamos, agora, ao entendimento do script Python responsável pela interação com o Turtlesim:</p>

<p>As linhas de código abaixo importam as bibliotecas e módulos necessários para a execução do projeto. <code>#!/usr/bin/env python3</code> especifica o interpretador necessário para rodar o script. O módulo <code>time</code> será utilizado para trabalhar com o intervalo de tempo entre a rotação e a movimentação linear da tartaruga. A biblioteca <code>rclpy</code> concretiza o interfaceamento entre Python e ROS. A classe <code>Node</code> representa um nó na rede ROS, enquanto <code>Twist</code> se refere ao tipo de dado utilizado pelo ROS para transmitir informações sobre a movimentação de um robô.</p>
<div align="center"><img src="https://github.com/amandafontes/M6-Inteli/blob/main/turtlesim_simulation/imagens/modules.png?raw=true" width="50%" height=auto></img></div>

<p>Abaixo, é criada a classe <code>TurtleController</code>, que herda da classe <code>Node</code>. Na classe, criamos as funções <code>__init__</code>, que configura alguns atributos por meio dos quais será possível manipular a simulação, e <code>move_turtle</code>, em que projetamos de que forma a tartaruga irá se movimentar na interface. Na primeira função, evidencia-se a existência de um sistema publisher-subscriber. O publisher enviará mensagens Twist para que os movimentos sejam realizados. Já o timer criado irá chamar a função de movimentação em intervalos de 0.1 segundo. Na segunda função, utilizam-se a velocidade linear e a velocidade angular para determinar como a tartaruga irá se comportar. Foi utilizado um loop de onze iterações para que os movimentos fossem executados, e um ângulo de rotação de 45º, determinado pela velocidade angular de 2.84. Ao final do ciclo, uma estrela de onze pontas será formada, e ambas as velocidades envolvidas serão configuradas para um valor aproximado de zero, para que a movimentação seja interrompida. Desse modo, o timer criado também é cancelado.</p>
<div align="center"><img src="https://github.com/amandafontes/M6-Inteli/blob/main/turtlesim_simulation/imagens/class.png?raw=true" width="60%" height=auto></img></div>

<p>Por fim, é criada a função principal, que será mantida em execução ao final do código. Ela inicializa o interfaceamento entre o script e o ROS, cria uma instância para a classe <code>TurtleController()</code> e mantém o nó correspondente em execução até que ele seja encerrado. Quando encerrado, os recursos utilizados pelo nó são anulados e a rede ROS é interrompida.</p>
<div align="center"><img src="https://github.com/amandafontes/M6-Inteli/blob/main/turtlesim_simulation/imagens/main.png?raw=true" width="60%" height=auto></img></div>

<p>Pronto! Agora, você deverá visualizar a tartaruga se movimentando de modo a desenhar uma estrela de onze pontas. O script pode ser adaptado de modo a guiar a tartaruga a desenhar outras figuras.</p>
