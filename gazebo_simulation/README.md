<h2>Simulação de robôs móveis com Gazebo</h2>

<h3>Enunciado</h3>

<p>Crie um pacote em ROS capaz de interagir com uma simulação feita no Gazebo de modo que a plataforma simulada do turtlebot3 seja capaz de mover-se de maneira controlada.</p>

- Interagir com os tópicos e/ou serviços do turtlebot3 de modo a conseguir mandar comandos de velocidade e extrair dados de odometria.
- Conceber uma estrutura de dados capaz de armazenar a série de movimentos que devem ser feitos pelo robô para chegar no objetivo.
- Implementar uma rota pré-estabelecida.

<h3>Demonstração</h3>

https://github.com/amandafontes/M6-Inteli-Robot-Simulation/assets/77015911/ac4a776e-828a-4b6d-a5e1-43e5e84abcf0

<h3>Implementação</h3>

Para a construção da atividade em questão, foi necessária a construção de um script Python que, por sua vez, conta com um sistema publisher-subscriber. Para definir a trajetória a ser realizada pelo turtlebot, foi feita uma lista de pontos, posteriormente interpretados pelo publisher a fim de que cada parte da rota fosse devidamente percorrida. Para o controle de movimentação do robô, foi criada a classe <code>TurtleController</code>, que abriga as funções necessárias. A primeira função cria o publisher e o subscriber de acordo com seus tópicos correspondentes, o timer, e as variáveis relacionadas à dinâmica do robô, bem como os pontos da trajetória. Em sequência, <code>publisher_callback</code> constitui a função responsável pela dinâmica da movimentação, isto é, fazer com que o robô passe por todos os pontos da lista por meio de sua velocidade angular e de sua velocidade linear, considerando a inclinação necessária para que cada ponto seja atingido. Por fim, o subscriber, representado pela função <code>subscriber_callback</code>, utiliza o tópico de odometria existente no Gazebo para retornar suas coordenadas ao longo do percurso, assim como sua orientação quanto ao próximo ponto a ser percorrido.

Posteriormente à construção do script, foi criado um pacote em ROS que permite a interação direta entre a simulação e o arquivo Python. Para abrir a simulação, portanto, basta executar o comando abaixo:

```powershell
ros2 run gazebo_simulation atividade_gazebo
```

