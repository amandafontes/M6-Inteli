import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

print(image.shape) # Obtenção das dimensões da imagem

# Redimensionar a imagem por meio dos parâmetros (largura, altura)

rescaled_image = cv.resize(image, (320, 214))

# Redimensionar a imagem por meio do parâmetro scale

rescaled_image_2 = cv.resize(image, None, fx=2, fy=2) # fx e fy são os fatores de escala

# Exibição da imagem redimensionada

cv.imshow('Original cat', image)
cv.imshow('Rescaled cat', rescaled_image)
cv.imshow('Rescaled cat 2', rescaled_image_2)
cv.waitKey(0)
cv.destroyAllWindows()