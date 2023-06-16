import cv2 as cv # Biblioteca para ferramentas de visão computacional

# Captura do vídeo selecionado
input_video = cv.VideoCapture('./video/video.mp4')

# Confere se foi possível abrir o vídeo
if not input_video.isOpened():
    print("Erro ao abrir o arquivo de vídeo.")
    exit(1)

# Obtém as dimensões do vídeo selecionado
width  = int(input_video.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(input_video.get(cv.CAP_PROP_FRAME_HEIGHT))

# Configura o resultado final do vídeo
fourcc = cv.VideoWriter_fourcc(*'mp4v')
output_video = cv.VideoWriter('./output.mp4', fourcc, 20.0, (width, height)) # 20.0 é o frame rate

# Loop para percorrer o vídeo, frame a frame
while True:

    ret, frame = input_video.read()

    # Interrupção do loop caso a leitura do vídeo selecionado não tenha sido bem-sucedida
    if not ret:
        break

    # Carrega os classificadores em cascata para detecção de faces
    face_cascade = cv.CascadeClassifier(filename=f'{cv.data.haarcascades}/haarcascade_frontalface_default.xml')

    # Lê o frame e passa pelo classificador de faces
    face_detection = face_cascade.detectMultiScale(
        image=frame, # Parâmetro que define qual imagem passará pelo filtro. No caso, o frame do vídeo
        scaleFactor=1.3, # Parâmetro que define o quanto a imagem será reduzida a cada escala
        minNeighbors=5) # Parâmetro que define quantos vizinhos cada retângulo candidato deve ter para ser mantido

    # Desenha os retângulos nas faces detectadas
    for (x, y, w, h) in face_detection: # x e y são as coordenadas do canto superior esquerdo do retângulo, w e h são a largura e altura do retângulo
        cv.rectangle(
            img=frame,
            pt1=(x, y), # pt1 e pt2 são os pontos que definem o retângulo. pt1 é o canto superior esquerdo e pt2 é o canto inferior direito
            pt2=(x + w, y + h), # pt2 é calculado a partir de pt1 somando-se a largura e altura do retângulo
            color=(255, 0, 0), # Cor do retângulo em BGR
            thickness=4 # Espessura da linha do retângulo
        )

    # Exibe o frame com a detecção
    # cv.imshow('Video', frame)

    # Acrescenta o frame ao arquivo em que será armazenado o vídeo final
    output_video.write(frame)

    # Aguarda que a tecla 'q' seja pressionada para encerrar o loop
    if cv.waitKey(30) & 0xFF == ord('q'):
        break

# Libera a captura de vídeo e fechando as janelas
output_video.release()
cv.destroyAllWindows()