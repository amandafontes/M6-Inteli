<h2>Processamento de imagens e detecção de objetos</h2>

<h3>Enunciado</h3>

<p>Desenvolva um script em Python capaz de identificar rachaduras em paredes de concreto. Utilize o dataset desenvolvido pela Roboflow. Para o desenvolvimento dessa atividade, recomenda-se o uso de um modelo de detecção de objetos pré-treinado, como o YoLo.</p>
  
<h3>Demonstração</h3>

<p>Abaixo, encontra-se um vídeo demonstrativo referente às imagens submetidas ao modelo, exibindo a detecção e a identificação das rachaduras.</p>

https://github.com/amandafontes/M6-Inteli-Robot-Simulation/assets/77015911/3bf783b8-58fa-4e5a-90e9-83e31496216a
  
<h3>Implementação</h3>
  
<p>Para a construção da atividade, foi utilizado o YoloV8. A atividade foi realizada em um Jupyter Notebook, que pode ser acessado no presente diretório. Abaixo, encontram-se descritos os procedimentos utilizados para a elaboração do modelo de detecção.</p>

<h4>Importações necessárias</h4>

<p>A primeira célula do arquivo <code>crack_detection.ipynb</code> realiza a instalação do <code>ultralytics</code>, por meio da qual será utilizada a ferramenta Yolo, e <code>roboflow</code>, da qual foi obtido o dataset utilizado para o treinamento do modelo.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/images/imports.png?raw=true" width=50%></img></p>

<h4>Setup do modelo pré-treinado</h4>

<p>Foi utilizado o <a href="https://universe.roboflow.com/university-bswxt/crack-bphdr/dataset/2">Crack Image Dataset</a>, do Roboflow. Abaixo, encontra-se a célula de código responsável por sua obtenção, além de uma validação que solicita a API Key do usuário.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/images/dataset.png?raw=true" width=60%></img></p>

<h4>Treinamento do modelo para o dataset utilizado</h4>

<p>A célula seguinte será responsável por atribuir o modelo pré-treinado do Yolo a uma variável, <code>model</code>. Posteriormente, será possível rodar o comando necessário para o treinamento do modelo, passando os parâmetros necessários.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/images/model.png?raw=true" width=65%></img></p>

<p>O código acima insere o comando diretamente na célula. Contudo, é possível rodá-lo diretamente no terminal. Uma outra alternativa para rodar o modelo na célula de um Jupyter Notebook ou em um script Python encontra-se abaixo:</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/images/model2.png?raw=true" width=65%></img></p>

<h4>Exibição de resultados</h4>

<p>Uma vez que o modelo é executado, os resultados obtidos ficam dispostos no diretório <code>runs</code>, que é criado automaticamente. Para exibir o resultado da detecção de rachaduras sobre uma amostra das imagens de teste, basta executar a célula abaixo, selecionando um dos arquivos que contêm as imagens resultantes.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/images/display.png?raw=true" width=65%></img></p>

<p>Abaixo, encontram-se alguns exemplos de predições realizadas pelo modelo.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/yolo_detection_model/runs/detect/train/val_batch0_labels.jpg?raw=true" width=60%></img></p>
