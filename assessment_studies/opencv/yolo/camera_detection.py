import cv2 as cv # Biblioteca para ferramentas de visão computacional
from ultralytics import YOLO # Biblioteca para detecção de objetos

# Carregando o modelo de detecção de objetos do Yolo
model = YOLO("./yolov8n.pt") 

# Inicializando a captura de vídeo
camera = cv.VideoCapture(0)

# Loop para captura dos frames se o dispositivo estiver em execução
if camera.isOpened():

    # Capturando o vídeo frame a frame
    ret, frame = camera.read() # ret é um booleano que retorna true se o frame for lido corretamente

    while True:

        detection = model(frame) # Aplicando o modelo de detecção de objetos no frame

        annotated_frame = detection[0].plot() # Guardando o frame com as detecções

        detection = model.predict(frame, conf=0.8, stream=True) # Realizando a predição do modelo no frame

        # Desenhando a caixa delimitadora
        for result in detection:
            
            # Obtendo as coordenadas dos retângulos das caixas delimitadoras
            boxes = result.boxes.cpu().numpy()

            # Iterando sobre as caixas delimitadoras
            for box in boxes:
                r = box.xyxy[0].astype(int)

                cv.rectangle(frame, r[:2], r[2:], (150, 55, 200), 2) # Desenhando o retângulo no frame

            # Definindo a fonte e escrevendo o nome do objeto detectado
            font = cv.FONT_HERSHEY_DUPLEX
            cv.putText(
                frame,
                result.names[int(box.cls[0])],
                (r[0] + 6, r[1] - 20),
                font,
                1.0,
                (0, 0, 0),
                1,
            )

        # Exibir o frame capturado
        cv.imshow('Webcam', frame)

        # Aguardando a tecla 'q' ser pressionada para sair do loop
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

# Liberando a captura de vídeo e fechando as janelas
camera.release()
cv.destroyAllWindows()