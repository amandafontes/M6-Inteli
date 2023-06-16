import time # Biblioteca para ferramentas de tempo
import cv2 as cv # Biblioteca para ferramentas de visão computacional
from ultralytics import YOLO # Biblioteca para detecção de objetos

# Carregando o modelo de detecção de objetos do Yolo
model = YOLO("./yolov8n.pt") 

# Capturando o vídeo da câmera
camera = cv.VideoCapture(0)

height = int(camera.get(cv.CAP_PROP_FRAME_HEIGHT)) # Obtendo a altura do frame
width = int(camera.get(cv.CAP_PROP_FRAME_WIDTH)) # Obtendo a largura do frame

# Definindo o codec e criando o objeto VideoWriter
fourcc = cv.VideoWriter_fourcc(*'mp4v')
video = cv.VideoWriter('output.mp4', fourcc, 20.0, (width, height)) # 20.0 é o frame rate

# Loop para aplicação do modelo de detecção de objetos ao vivo
while True:
    _, frame = camera.read() # Capturando o frame da câmera

    # Realizando a predição com índice de confiança de 0.6
    detection = model.predict(frame, conf=0.6)

    # Exibindo o frame com as detecções
    cv.imshow('Webcam', detection[0].plot())
    video.write(detection[0].plot())

    # Interrompendo o loop se a tecla 'q' for pressionada
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.03)

# Liberando a captura de vídeo e fechando as janelas
video.release()
camera.release()
cv.destroyAllWindows()