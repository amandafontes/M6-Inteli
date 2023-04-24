<h2>Turtlesim: simulando um ambiente robótico integrado no ROS</h2>

<h3>Enunciado</h3>

<p>Crie um script em Python capaz de interagir com o nó de simulação do turtlesim e enviar mensagens nos tópicos que regem a locomoção da tartaruga principal. Utilize este script para reproduzir um desenho de sua autoria. Utilize a estrutura de dados que preferir para representar a “imagem” a ser desenhada. O uso de programação orientada a objetos é obrigatório.</p>

<h3>Implementação</h3>

<p>Para a construção da atividade, será necessário, primeiramente, instalar e configurar o Windows Subsystem for Linux (WSL). Posteriormente, o mesmo procedimento deve ser feito para o Robot Operating System (ROS), sistema operacional que será utilizado para construir a simulação.</p>

<p>Uma vez configurado o ambiente de desenvolvimento, é necessário abrir o script Python, localizado no diretório <code>turtlesim_simulation/turtlesim.py</code>. O código é responsável por interagir com a aplicação correspondente ao Turtlesim, e irá fornecer as instruções necessárias para que a tartaruga se movimente a fim de completar a figura geométrica que queremos obter.</p>

<p>Sabendo-se que foi criado um ambiente virtual Python especialmente para a simulação, será necessário configurá-lo. Primeiramente, vá até o diretório <code>turtlesim_simulation</code>, abra o terminal, no WSL, e digite o seguinte comando:</p>

<code>pip install requirements.txt</code>

<p>Agora que você instalou os módulos e pacotes utilizados no diretório, deve ativar o ambiente virtual Python criado. Isso pode ser feito por meio do seguinte comando:</p>

<code>source turtlesim_env/bin/activate</code>

<p>O próximo passo é segmentar o terminal em duas partes. Em um terminal, abriremos o Turtlesim. No outro, iremos executar o script Python. No primeiro terminal, digite o seguinte comando:</p>

<code>python3 turtlesim.py</code>

<p>No segundo terminal, digite o seguinte comando:</p>

<code>ros2 run turtlesim turtlesim_node</code>

<p>Pronto! Agora, você deverá visualizar a tartaruga se movimentando de modo a desenhar uma estrela de onze pontas. Abaixo, encontra-se um vídeo demonstrativo da atividade em funcionamento.</p>

https://user-images.githubusercontent.com/77015911/234037806-c4661724-a9df-47ca-83de-ec9be10e01dd.mp4



