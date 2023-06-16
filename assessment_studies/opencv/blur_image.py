import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Desfocando a imagem utilizando o filtro gaussiano com kernel 7x7

blurred_image = cv.GaussianBlur(image, (7, 7), 0)

# Exibindo a imagem desfocada

cv.imshow('Imagem desfocada', blurred_image)
cv.imshow('Imagem original', image)
cv.waitKey(0)
cv.destroyAllWindows()