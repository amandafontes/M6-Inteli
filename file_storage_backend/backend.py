import os # Biblioteca para manipulação de arquivos
import time # Biblioteca para capturar o tempo em que a imagem é enviada
import ultralytics # Biblioteca para o modelo de visão computacional aplicado às imagens
from ultralytics import YOLO # Biblioteca para aplicação do modelo YOLO
from flask import Flask, request, render_template # Biblioteca para criar o servidor em Flask
from supabase import create_client, Client # Biblioteca para conectar no supabase
from shutil import rmtree # Biblioteca para remoção de diretórios temporários

# Cria o servidor
app = Flask(__name__) 

# Chave de acesso ao supabase
url = "https://exwsjbueckvbqbvtahzr.supabase.co"
key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImV4d3NqYnVlY2t2YnFidnRhaHpyIiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTY4NjY2NjYxMCwiZXhwIjoyMDAyMjQyNjEwfQ.k5B0bBc3ttbSlnvOzqQZldVQZYcOf7cxum9Eg3blavI"

# Cria o cliente para conectar na API do supabase
supabase: Client = create_client(url, key)

# Estabelece o nome do bucket
bucket_name = "images"

# Rota para realizar upload de imagens
@app.route("/upload", methods=["GET", "POST"])
def upload_image():

    if request.method == "POST":

        # Pega o arquivo correspondente ao upload que o usuário realizou via interface
        image_file = request.files["image"]

        # Cria um nome para o arquivo correspondente à imagem temporária
        image_name = f"temporary_image_{time.time()}.jpg"

        # Salva a imagem em um arquivo temporário para que seja processada pelo modelo de detecção de objetos
        image_path = f"./temporary_image/{image_name}"
        
        # Cria o diretório para armazenar a imagem temporária
        os.makedirs(os.path.dirname(image_path), exist_ok=True)
        
        # Lê o arquivo de imagem recebido
        f = open(f"{image_path}", "wb")
        f.write(image_file.read())

        # Carrega o modelo YOLO utilizado para a detecção de objetos
        model = YOLO('yolov8n.pt')

        # Roda o modelo de detecção de objetos sobre a imagem enviada
        prediction = model.predict(image_path, conf=0.6, save=True)
        predicted_image_path = f"./runs/detect/predict/{image_name}"

        # Lê o arquivo de imagem recebido
        image_data = open(predicted_image_path, 'rb').read()

        # Cria um padrão de nome para as imagens enviadas
        filename = f"{time.time()}_{image_file.filename}"

        # Envia a imagem para o bucket do supabase
        response = supabase.storage.from_(bucket_name).upload(filename, image_data)
        print(response)

        # Limpa os diretórios temporários para cada imagem enviada via interface
        rmtree("./runs")
        rmtree("./temporary_image")

        # Confere se o upload foi concretizado
        if response.status_code == 200:
            return "A imagem foi enviada com sucesso."
        else:
            return "O upload da imagem falhou."

    # Retornando a página de upload
    return render_template("upload.html")

if __name__ == '__main__':
    app.run(debug=True)