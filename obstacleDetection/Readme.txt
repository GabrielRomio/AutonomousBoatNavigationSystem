EN
=========================================================================================================
ALGORITHM FOR DETECTION AND CLASSIFICATION OF OBSTACLES

The following algorithm runs exclusively on the external computer. Its purpose is to 
capture images from the infrared camera, generate a depth map, and apply filters to locate 
and classify obstacles that may impede the vehicle's navigation. It generates a vector of 
five positions, each corresponding to the proximity of obstacles in one of the image regions, 
and sends them to the Raspberry Pi via UDP sockets.

This code is registered with the Brazilian National Institute of Industrial Property - INPI, under the following information:

Title: "ALGORITMO PARA DETECÇÃO E CLASSIFICAÇÃO DOS OBSTÁCULOS"
Author: Romio, G.; Registration number: BR512022003232-5; Registration date: 29/11/2022;

Therefore, it should only be used for reference or academic purposes.
=========================================================================================================

PT-BR
=========================================================================================================
ALGORITMO PARA DETECÇÃO E CLASSIFICAÇÃO DOS OBSTÁCULOS 

O  algoritmo  a  seguir  é  o  único  a  rodar  no  computador  externo.  Tem  como 
objetivo  capturar  as  imagens  da  câmera  de  infravermelho,  gerar  o  mapa  de 
profundidade e aplicar filtros para localizar e classificar obstáculos à navegação do 
veículo.  Assim,  gera  um  vetor  de  cinco  posições,  cada  um  deles  referente  à 
proximidade dos obstáculos em uma das regiões da imagem, e os envia ao Raspberry 
PI por meio de sockets UDP.

Está código está registrado no Instituto Nacional da Propriedade Industrial - INPI, sob os seguintes dados:

Título: "ALGORITMO PARA DETECÇÃO E CLASSIFICAÇÃO DOS OBSTÁCULOS"
Autor: Romio, G.; Número do registro: BR512022003232-5; Data de registro: 29/11/2022;

Assim, deve ser usado apenas para consultas ou fins acadêmicos.
=========================================================================================================