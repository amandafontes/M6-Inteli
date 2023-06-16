import cv2 as cv # Biblioteca para ferramentas de visão computacional
import numpy as np # Biblioteca para manipulação de arrays

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Rotacionando a imagem em 90 graus

rotated_image = cv.rotate(image, cv.ROTATE_90_CLOCKWISE)

# Exibindo a imagem rotacionada

cv.imshow('Rotated cat', rotated_image)
cv.waitKey(0)
cv.destroyAllWindows()