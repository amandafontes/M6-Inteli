import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

cv.imshow('Cat', image) # Exibe a imagem lida anteriormente

cv.waitKey(0) # Espera o usuário pressionar uma tecla para fechar a janela
cv.destroyAllWindows() # Fecha todas as janelas abertas pelo programa