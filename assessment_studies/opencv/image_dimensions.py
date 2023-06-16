import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Obtenção das dimensões da imagem

dimensions = image.shape # Retorna uma tupla com as dimensões da imagem (linhas, colunas, canais)

height = image.shape[0] # Altura da imagem
width = image.shape[1] # Largura da imagem
channels = image.shape[2] # Canais da imagem (3 para RGB, 4 para RGBA)

# Exibição das dimensões da imagem

print('Dimensão da imagem:', dimensions)
print('Altura da imagem:', height)
print('Largura da imagem:', width)
print('Canais da imagem:', channels)

# Exibição da própria imagem

cv.imshow('Cat', image) # Exibe a imagem lida anteriormente

cv.waitKey(0) # Espera o usuário pressionar uma tecla para fechar a janela
cv.destroyAllWindows() # Fecha todas as janelas abertas pelo programa