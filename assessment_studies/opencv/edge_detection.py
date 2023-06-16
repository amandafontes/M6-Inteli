import cv2 as cv # Biblioteca para ferramentas de visão computacional

image = cv.imread('./images/Photos/cat.jpg') # Lê a imagem

# Detectando as bordas da imagem utilizando o operador de Canny

image_edges = cv.Canny(image, 100, 200)

# Exibindo a imagem com as bordas detectadas

cv.imshow('Imagem com bordas detectadas', image_edges)
cv.imshow('Imagem original', image)
cv.waitKey(0)
cv.destroyAllWindows()