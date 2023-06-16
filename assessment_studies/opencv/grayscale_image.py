import cv2 as cv # Biblioteca para ferramentas de visão computacional
import numpy as np # Biblioteca para manipulação de arrays

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

print(image.shape) # Obtenção das dimensões da imagem

# Converter a imagem para escala de cinza

gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# Exibição da própria imagem

cv.imshow('Gray cat', gray_image) # Exibe a imagem lida anteriormente

cv.waitKey(0) # Espera o usuário pressionar uma tecla para fechar a janela
cv.destroyAllWindows() # Fecha todas as janelas abertas pelo programa