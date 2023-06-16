<h2>Atividade prática | P2</h2>

<h3>Enunciado</h3>

Desenvolva um código em Python capaz de utilizar o openCV para a leitura de um vídeo (frame a frame) e, para cada frame, o seu código deve identificar e marcar na imagem os retângulos correspondentes a cada uma das faces encontradas. Ao final do código, um novo vídeo deve ser salvo com a(s) face(s) identificada(s).

<h3>Implementação</h3>

Para o desenvolvimento da atividade prática correspondente à segunda prova do módulo 6, foram adotados os seguintes procedimentos: leitura e segmentação do vídeo frame a frame, por meio de métodos da biblioteca <code>opencv</code>, implementação de sistema de reconhecimento facial por meio do filtro <code>Haar Cascade</code> e posterior armazenamento do vídeo contendo todas as detecções realizadas.

Primeiramente, foi realizada a leitura do vídeo, e suas dimensões foram capturadas. Posteriormente, atribuímos a <code>output_video</code> a configuração final do vídeo, ou seja, contendo o conjunto de frames após o processamento pelo filtro de detecção utilizado. O próximo passo foi a criação de um loop capaz de percorrer o vídeo selecionado (<code>input_video</code>), frame a frame. Foram também carregados os classificadores em castaca para detecção de faces, por meio do método <code>CascadeClassifier</code>. Desse modo, cada frame foi submetido ao classificador por meio do método <code>detectMultiScale</code>. A variável <code>face_detection</code> armazena o processo em questão.

O passo seguinte se refere ao desenho do retângulo sobre a face encontrada no frame. Isso é feito por meio do loop <code>for (x, y, w, h) in face_detection:</code>, em que são passados os parâmetros necessários para a concretização do desenho. Posteriormente, há a possibilidade de exibirmos cada um dos frames à medida que ele passa pelo processo de detecção. Por fim, o <code>output_video</code> armazena o frame para a construção do vídeo final. Para encerrar o processo, liberamos o processo relacionado à leitura do vídeo final e fechamos quaisquer janelas que possam estar abertas. O vídeo final é armazenado no mesmo diretório em que o script é executado.

<h3>Demonstração</h3>

Um vídeo demonstrativo do script em funcionamento pode ser visualizado abaixo:

https://github.com/amandafontes/M6-Inteli-Robot-Simulation/assets/77015911/a29d7fa3-5a77-457a-a91f-fef8739e789d
