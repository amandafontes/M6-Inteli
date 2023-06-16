import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

print(image.shape) # Obtenção das dimensões da imagem

# Recortar a imagem

cropped_image = image[27:427,255:640] # Passa o intervalo das coordenadas que deseja recortar, em ordem: [y1:y2, x1:x2]

# Exibição da própria imagem

cv.imshow('Cropped cat', cropped_image) # Exibe a imagem lida anteriormente

cv.waitKey(0) # Espera o usuário pressionar uma tecla para fechar a janela
cv.destroyAllWindows() # Fecha todas as janelas abertas pelo programa