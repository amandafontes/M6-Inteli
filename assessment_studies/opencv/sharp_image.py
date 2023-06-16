import cv2 as cv # Biblioteca para ferramentas de visão computacional
import numpy as np # Biblioteca para manipulação de arrays

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Criando um kernel de nitidez

kernel = np.array([[0, 0, 0],
                   [-1, 5, -1],
                   [0, -1, 0]], dtype=np.float32)

# Aplicando a convolução

sharp_image = cv.filter2D(image, -1, kernel) # Parâmetros: imagem, profundidade, kernel

# Exibindo da imagem com nitidez modificada

cv.imshow('Sharp cat', sharp_image)
cv.imshow('Cat', image)
cv.waitKey(0)
cv.destroyAllWindows()