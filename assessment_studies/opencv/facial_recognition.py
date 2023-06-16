import cv2 as cv # Biblioteca para ferramentas de visão computacional

# Carregando os classificadores em cascata para detecção de faces e olhos

face_cascade = cv.CascadeClassifier(filename=f'{cv.data.haarcascades}/haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier(filename=f'{cv.data.haarcascades}/haarcascade_eye.xml')

# Lendo a imagem e passando pelo classificador de faces e olhos

image = cv.imread('./images/Photos/lady.jpg')

face_detection = face_cascade.detectMultiScale(
    image=image,
    scaleFactor=1.3, # Parâmetro que define o quanto a imagem será reduzida a cada escala
    minNeighbors=5) # Parâmetro que define quantos vizinhos cada retângulo candidato deve ter para ser mantido

# Desenhando os retângulos nas faces detectadas

for (x, y, w, h) in face_detection: # x e y são as coordenadas do canto superior esquerdo do retângulo, w e h são a largura e altura do retângulo
    cv.rectangle(
        img=image,
        pt1=(x, y), # pt1 e pt2 são os pontos que definem o retângulo. pt1 é o canto superior esquerdo e pt2 é o canto inferior direito
        pt2=(x + w, y + h), # pt2 é calculado a partir de pt1 somando-se a largura e altura do retângulo
        color=(255, 0, 0), # Cor do retângulo em BGR
        thickness=1 # Espessura da linha do retângulo
    )

    # Passando a região da face detectada pelo classificador de faces para o classificador de olhos

    face_region = image[y:y + h, x:x + w]
    eyes_detection = eye_cascade.detectMultiScale(face_region)

    # Desenhando os retângulos nos olhos detectados

    for (ex, ey, ew, eh) in eyes_detection:
        cv.rectangle(
            img=face_region,
            pt1=(ex, ey),
            pt2=(ex + ew, ey + eh),
            color=(0, 255, 0),
            thickness=1
        )

# Exibindo a imagem com as faces e olhos detectados

cv.imshow('Imagem com faces e olhos detectados', image)
cv.waitKey(0)
cv.destroyAllWindows()