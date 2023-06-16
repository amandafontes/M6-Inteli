import cv2 as cv # Biblioteca para ferramentas de visão computacional
import numpy as np # Biblioteca para manipulação de arrays

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Espelhando a imagem horizontalmente

flipped_image = cv.flip(image, 1) # 1 = horizontalmente, 0 = verticalmente, -1 = horizontalmente e verticalmente

# Exibindo a imagem espelhada

cv.imshow('Rotated cat', flipped_image)
cv.waitKey(0)
cv.destroyAllWindows()