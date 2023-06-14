<h2>Backend para transmissão e armazenamento de imagens</h2>

<h3>Enunciado</h3>

<p>Desenvolva o software de um backend capaz de receber imagens e armazená-las adequadamente. Não há restrições com relação à tecnologia utilizada.</p>
  
<h3>Demonstração</h3>

<p>Abaixo, encontra-se um vídeo demonstrativo do artefato em funcionamento. No teste realizado, foi feito o upload de uma imagem por meio da interface criada. A imagem em questão foi recebida pelo backend, armazenada temporariamente, submetida ao modelo de detecção de objetos e, por fim, enviada a um bucket do Supabase. Ao acessar o bucket, é possível visualizar as imagens enviadas para o gerenciador de arquivos, posteriormente ao processamento realizado pelo modelo.</p>

*vídeo_demonstrativo*

<h3>Implementação</h3>

<p>Para a realização da atividade, foram utilizados Flask e um modelo de detecção de objetos proveniente do YoloV8. Abaixo, encontram-se descritos os procedimentos utilizados para a elaboração do backend.</p>

<h4>Interface para upload de imagens</h4>

<p>A fim de possibilitar o upload de imagens para processamento e envio ao bucket, foi criada uma interface simples em <code>html</code>. Posteriormente, será possível visualizar o mecanismo utilizado para capturar a imagem no backend.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/html.png?raw=true" width=75%></img></p>

<h4>Importações necessárias</h4>

<p>O código produzido depende de algumas bibliotecas que são de fundamental importância para sua execução: <code>os</code>, para manipulação de arquivos, <code>time</code>, para a captura de tempo que compõe o nome dos arquivos, e <code>ultralytics</code> para a obtenção do modelo de visão computacional utilizado. Além disso, foram utilizados os módulos <code>rmtree</code>, para a remoção de diretórios temporários (seu uso será melhor evidenciado posteriormente), <code>YOLO</code>, que configura o modelo de detecção de objetos aplicado às imagens recebidas e <code>create_client</code> e <code>Client</code> para o setup correto do Supabase. Por fim, <code>Flask</code>, <code>request</code> e <code>render_template</code> foram importados a fim de configurar o servidor e as rotas.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/imports.png?raw=true" width=75%></img></p>

<h4>Configuração do servidor e do Supabase</h4>

<p>Primeiramente, inicializamos a aplicação em Flask. Posteriormente, definem-se <code>url</code>, o endereço para o projeto no Supabase, e a <code>key</code>, chave de acesso ao projeto. Na linha abaixo, inicializamos o Supabase client, de modo a possibilitar a interação entre a aplicação e o ecossistema do file storage. Por fim, atribuímos a <code>bucket_name</code> o nome do bucket criado.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/setup.png?raw=true" width=75%></img></p>

<h4>Criação da rota e tratamento inicial da imagem recebida</h4>

<p>Abaixo, encontra-se a parte do código em que a rota de upload foi construída. Uma vez que a rota recebe uma requisição, a função <code>upload_image</code> é executada. Primeiramente, ocorre uma verificação para conferir se o método de requisição recebido é do tipo <code>POST</code>. Uma vez que a verificação é feita, a imagem recebida através da interface é armazenada em uma variável. As linhas de código seguintes atribuem um nome e um caminho padronizados ao arquivo recebido.</p>

<p>Considerando que, antes de ser submetida ao modelo de detecção de objetos, a imagem deve ser armazenada temporariamente, o comando <code>os.makedirs</code> estabelece a criação de um diretório para seu armazenamento. Por fim, a imagem, inicialmente tratada, é salva em seu respectivo caminho, para enfim passar pelo processamento do modelo de visão computacional do Yolo.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/route.png?raw=true" width=75%></img></p>

<h4>Aplicação do modelo de visão computacional</h4>

<p>O bloco de código abaixo evidencia o momento em que a imagem é, de fato, processada pelo modelo. Primeiramente, atribuímos a <code>model</code> o modelo do YoloV8 utilizado, cujo arquivo se encontra no mesmo diretório em que está contido o script. Posteriormente, o método <code>predict</code> é utilizado para a identificação de objetos na imagem. Foram passados, como parâmetros, o caminho para a imagem (<code>image_path</code>), o índice de confiança da predição (60%) e a condição para salvar a imagem.</p>

<p>Uma vez que a imagem passa pelo processo de predição, o arquivo correspondente recebe um caminho e é armazenado em <code>image_data</code>. Por fim, ocorre a padronização do nome do arquivo já processado, de modo que seu upload seja concretizado no bucket do Supabase. A linha de código <code>supabase.storage.from_(bucket_name).upload(filename, image_data)</code>, que conta com os métodos necessários para a conclusão desse procedimento, é atribuída à variável <code>response</code>. Na linha seguinte, ocorre a impressão do resultado desse procedimento no terminal, a fim de que seja obtida uma resposta quanto ao sucesso da operação.</p>

<p>O método <code>rmtree</code>, posteriormente ao upload da imagem processada no file storage, é executado sobre os diretórios responsáveis pelo armazenamento temporário da imagem. Desse modo, a imagem é guardada no bucket, mas o ambiente em que se encontra o script se torna livre do arquivo.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/prediction.png?raw=true" width=75%></img></p>

<h4>Verificação do envio da imagem</h4>

<p>As últimas linhas de código da rota evidenciam uma verificação do <code>status_code</code> do procedimento realizado. Caso a operação tenha sido bem-sucedida, o usuário é redirecionado a uma mensagem de confirmação. Caso contrário, uma mensagem de erro é apresentada.</p>

<p>O retorno da página de upload via <code>render_template</code> fecha a rota e, por fim, a aplicação é devidamente inicializada.</p>

<p align="center"><img src="https://github.com/amandafontes/M6-Inteli-Robot-Simulation/blob/main/file_storage_backend/code_documentation/status.png?raw=true" width=75%></img></p>
