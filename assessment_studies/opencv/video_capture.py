import cv2 as cv # Biblioteca para ferramentas de visão computacional
from ultralytics import YOLO # Biblioteca para detecção de objetos

# Inicializando a captura de vídeo
camera = cv.VideoCapture(0)

# Loop para captura dos frames se o dispositivo estiver em execução
while camera.isOpened():

    # Capturando o vídeo frame a frame
    ret, frame = camera.read() # ret é um booleano que retorna true se o frame for lido corretamente

    # Exibir o frame capturado
    cv.imshow('frame', frame)

    # Aguardando a tecla 'q' ser pressionada para sair do loop
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Liberando a captura de vídeo e fechando as janelas
camera.release()
cv.destroyAllWindows()