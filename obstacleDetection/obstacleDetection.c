/*ALGORITHM FOR DETECTION AND CLASSIFICATION OF OBSTACLES
	The following algorithm runs exclusively on the external computer. Its purpose is to 
capture images from the infrared camera, generate a depth map, and apply filters to locate 
and classify obstacles that may impede the vehicle's navigation. It generates a vector of 
five positions, each corresponding to the proximity of obstacles in one of the image regions, 
and sends them to the Raspberry Pi via UDP sockets. 
	The code uses the DUO3D camera, as well as the libraries provided by the manufacturer to 
acquire images from it.*/

/*ALGORITMO PARA DETECÇÃO E CLASSIFICAÇÃO DOS OBSTÁCULOS 
	O  algoritmo  a  seguir  é  o  único  a  rodar  no  computador  externo.  Tem  como 
objetivo  capturar  as  imagens  da  câmera  de  infravermelho,  gerar  o  mapa  de 
profundidade e aplicar filtros para localizar e classificar obstáculos à navegação do 
veículo.  Assim,  gera  um  vetor  de  cinco  posições,  cada  um  deles  referente  à 
proximidade dos obstáculos em uma das regiões da imagem, e os envia ao Raspberry 
PI por meio de sockets UDP. 
	O código utiliza a câmera DUO3D, bem como as bibliotecas disponibilizadas pelo fabricante
para aquisição de imagens da mesma.*/
 
#include "Sample.h"   
#include <stdio.h>   
#include <stdlib.h>   
#include <unistd.h>   
#include <arpa/inet.h>   
#include <string.h>   
   
//Definição da resolução das imagens e frames por segundo   
#define WIDTH   320   
#define HEIGHT  240   
#define FPS     30   
  
//Endereço para comunicação através de UDP   
#define PORTSUPERVISORIO 2302   
  
//Estrutura para geração de mapa de profundidade colorido, onde cores   
//quentes representam objetos próximos e cores frias objetos distantes   
//Não será usado no projeto   
Vec3b HSV2RGB(float hue, float sat, float val)   
{   
   float x, y, z;   
   if(hue == 1) hue = 0;   
   else         hue *= 6;   
  
   int i = static_cast<int>(floorf(hue));   
   float f = hue - i;   
   float p = val * (1 - sat);   
   float q = val * (1 - (sat * f));   
   float t = val * (1 - (sat * (1 - f)));   
  
   switch(i)   
   {   
       case 0: x = val; y = t; z = p; break;   
       case 1: x = q; y = val; z = p; break;   
       case 2: x = p; y = val; z = t; break;   
       case 3: x = p; y = q; z = val; break;   
       case 4: x = t; y = p; z = val; break;   
       case 5: x = val; y = p; z = q; break;   
   }   
   return Vec3b((uchar)(x * 255), (uchar)(y * 255), (uchar)(z * 255));   
}   
  
int main(int argc, char* argv[])   
{   
   //Variaveis referentes aos sockets   
   socklen_t socketsize; //Recebe o tamanho da estrutura sockaddr_in    
   int socket_supervisorio;   
   struct sockaddr_in endereco_supervisorio;   
  
   //Demais variáveis referentes à captura de imagens e aplicação de filtros   
   int cont=0;   
   int contFrames=0;   
   char saida[5];   
   unsigned char pixel[5];   
   Mat filter[5] = Mat(Size(WIDTH, HEIGHT), CV_8UC1);   
  
   //Variaveis para geração de mapa de profundidade colorido - Não será utilizado   
   Mat colorLut = Mat(cv::Size(256, 1), CV_8UC3);   
   for(int i = 0; i < 256; i++)   
       colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);   
  
   //Inicia câmera e captura de imagens   
   if(!OpenDUOCamera(WIDTH, HEIGHT, FPS))   
   {   
       printf("Could not open DUO camera\n");   
       return 1;   
   }   
  
   printf("DUOLib Version:       v%s\n", GetDUOLibVersion());   
   printf("Dense3D Version:      v%s\n", Dense3DGetLibVersion());   
  
   //Inicia bibliotecas para gerar mapa de profundidade   
   Dense3DInstance dense3d;   
   if(!Dense3DOpen(&dense3d))   
   {   
       printf("Could not open Dense3D library\n");   
       CloseDUOCamera();   
       return 1;   
   }   
  
   //Insere licença para utilização das bibliotecas   
   if(!SetDense3DLicense(dense3d, "XXXX-XXXX-XXXX-XXXX-XXXX"))   
   {   
       printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");   
       CloseDUOCamera();   
       Dense3DClose(dense3d);   
       return 1;   
   }   
  
   //Define dimensões das imagens   
   if(!SetDense3DImageSize(dense3d, WIDTH, HEIGHT))   
   {   
       printf("Invalid image size\n");   
       CloseDUOCamera();   
       Dense3DClose(dense3d);   
       return 1;   
   }   
  
   //Carrega parâmetros da calibração realizada   
   DUO_STEREO params;   
   if(!GetCameraStereoParameters(&params))   
   {   
       printf("Could not get DUO camera calibration data\n");   
       CloseDUOCamera();   
       Dense3DClose(dense3d);   
       return 1;   
   }   
  
   //Define parâmetros da câmera   
   SetDense3DScale(dense3d, 3);//3   
   SetDense3DMode(dense3d, 1);//0 //1    
   SetDense3DCalibration(dense3d, &params);   
   SetDense3DNumDisparities(dense3d, 2);//4   
   SetDense3DSADWindowSize(dense3d, 6);//6   
   SetDense3DPreFilterCap(dense3d, 28);//28   
   SetDense3DUniquenessRatio(dense3d, 27);//27   
   SetDense3DSpeckleWindowSize(dense3d, 52);//52   
   SetDense3DSpeckleRange(dense3d, 14);//14   
   //Define exposição, intensidade dos LEDs de infravermelho e orientação da câmera
   
   SetExposure(10);//Valores mais altos melhoram as imagens em abientes escuros 
   SetLed(20);//Ambiente escuro 28 / Ambiente claro 20   
   SetVFlip(false);   
   SetUndistort(true);   
  
   //Cria matrizes para armazenar imagens e mapas de profundidade   
   Mat1f disparity = Mat(Size(WIDTH, HEIGHT), CV_32FC1);   
   Mat3f depth3d = Mat(Size(WIDTH, HEIGHT), CV_32FC3);   
  
   //Inicializa sockets   
   socket_supervisorio = socket(AF_INET, SOCK_DGRAM,0);   
   socketsize = sizeof(struct sockaddr_in); //Guarda o tamanho da estrutura sockaddr_in   
   endereco_supervisorio.sin_family = AF_INET;   
   endereco_supervisorio.sin_addr.s_addr = inet_addr("192.168.1.199"); //IP do servidor para o qual enviara pacotes   
   endereco_supervisorio.sin_port=htons(PORTSUPERVISORIO); //Define a porta de escuta da conexão   
   memset(&(endereco_supervisorio.sin_zero),'\0',sizeof(endereco_supervisorio.sin_zero)); //Zera o resto da estrutura   
  
   //Inicia loop de captura das imagens   
   while((cvWaitKey(1) & 0xff) != 27)   
   {   
       //Captura frame   
       PDUOFrame pFrameData = GetDUOFrame();   
       if(pFrameData == NULL) continue;   
  
       //Cria matrizes para receber imagens da câmera   
       Mat left = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->leftData);   
       Mat right = Mat(Size(WIDTH, HEIGHT), CV_8UC1, pFrameData->rightData);   
  
       //Gera mapa de profundidade em tons de cinza   
       if(Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData, (float*)disparity.data, (PDense3DDepth)depth3d.data))   
       {   
           uint32_t disparities;   
           GetDense3DNumDisparities(dense3d, &disparities);   
           Mat disp8;   
           disparity.convertTo(disp8, CV_8UC1, 255.0/(disparities*16));   
  
           //Converte mapa de profundidade em tons de cinza para mapa de profundidade colorido   
           //Mat mRGBDepth;   
           //cvtColor(disp8, mRGBDepth, COLOR_GRAY2BGR);   
           //LUT(mRGBDepth, colorLut, mRGBDepth);   
  
           //Exibe janela com mapa de profundidade gerado   
           imshow("Dense3D Disparity Map Greyscale", disp8);   
           //Exibe janelas com imagens capturada pelas câmeras esquerda e direita   
           imshow("Left", left);   
           imshow("Right", right);   
  
  
           // === Inicia processamento de imagem ===   
  
           //Armazena os 5 últimos frames para aplicação de filtros    
           filter[4] = filter[3];   
           filter[3] = filter[2];   
           filter[2] = filter[1];   
           filter[1] = filter[0];   
           filter[0] = disp8;   
           contFrames = contFrames+1;   
  
           //Define matriz para receber dados do mapa de profundidade pós-filtro   
           Mat posfilter = Mat(Size(WIDTH, HEIGHT), CV_8UC1);   
           int NivelFiltro = 4; //Nível do filtro de 1 a 5   
           unsigned char auxpixel;   
  
           //Variável que indica o conjunto de pertinência de cada posição   
           //apenas para fins de monitoração   
           saida[0]=0;   
           saida[1]=0;   
           saida[2]=0;   
           saida[3]=0;   
           saida[4]=0;   
  
           //Variável que indica o tom de cinza referente à proximidade dos   
           //obstáculos de cada região da imagem   
           pixel[0]=0;   
           pixel[1]=0;   
           pixel[2]=0;   
           pixel[3]=0;   
           pixel[4]=0;   
  
           if (contFrames>=5){ //A cada 5 frames realiza o processamento da imagem 
  
               contFrames = 0;   
  
               //Analisa a imagem pixel a pixel para localizar e classificar possíveis obstáculos   
               for(int y=0; y<HEIGHT; y++){ //Percorre a imagem linha a linha   
                   for(int x=0; x<WIDTH; x++){ //Percorre a imagem coluna a coluna 
  
                       //Remove fundo e objetos muito distantes   
                       if (filter[0].at<uchar>(y,x)<50){   
                           posfilter.at<uchar>(y,x) = 0;   
  
                           auxpixel = filter[0].at<uchar>(y,x);   
                           //Grava cor do pixel referente à distância do objeto em variavel que será enviada ao controlador fuzzy   
                           if (x<=WIDTH*1/5 and pixel[0]<auxpixel)   
                               pixel[0]=auxpixel;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and pixel[1]<auxpixel)   
                               pixel[1]=auxpixel;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and pixel[2]<auxpixel)   
                               pixel[2]=auxpixel;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and pixel[3]<auxpixel)   
                               pixel[3]=auxpixel;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and pixel[4]<auxpixel)   
                               pixel[4]=auxpixel;   
                       }   
                       //Seleciona objetos muito distantes   
                       else if (((filter[0].at<uchar>(y,x)>=50 and filter[0].at<uchar>(y,x)<75) + (filter[1].at<uchar>(y,x)>=50 and filter[1].at<uchar>(y,x)<75) 
						+ (filter[2].at<uchar>(y,x)>=50 and filter[2].at<uchar>(y,x)<75) + (filter[3].at<uchar>(y,x)>=50 and filter[3].at<uchar>(y,x)<75) 
						+ (filter[4].at<uchar>(y,x)>=50 and filter[4].at<uchar>(y,x)<75))>=NivelFiltro){   
                           posfilter.at<uchar>(y,x) = 75;    
                           //Grava distância do objeto em variavel para monitoração
   
                           if (x<=WIDTH*1/5 and saida[0]<1)   
                               saida[0]=1;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and saida[1]<1)   
                               saida[1]=1;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and saida[2]<1)   
                               saida[2]=1;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and saida[3]<1)   
                               saida[3]=1;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and saida[4]<1)   
                               saida[4]=1;   
  
                           auxpixel = (filter[0].at<uchar>(y,x) + filter[1].at<uchar>(y,x) + filter[2].at<uchar>(y,x) + filter[3].at<uchar>(y,x) + filter[4].at<uchar>(y,x))/5;   
                           //Grava cor do pixel referente à distância do objeto em variavel que será enviada ao controlador fuzzy   
                           if (x<=WIDTH*1/5 and pixel[0]<auxpixel)   
                               pixel[0]=auxpixel;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and pixel[1]<auxpixel)   
                               pixel[1]=auxpixel;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and pixel[2]<auxpixel)   
                               pixel[2]=auxpixel;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and pixel[3]<auxpixel)   
                               pixel[3]=auxpixel;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and pixel[4]<auxpixel)   
                               pixel[4]=auxpixel;   
                       }   
                       //Seleciona objetos de distância média   
                       else if (((filter[0].at<uchar>(y,x)>=75 and filter[0].at<uchar>(y,x)<125) + (filter[1].at<uchar>(y,x)>=75 and filter[1].at<uchar>(y,x)<125) 
						+ (filter[2].at<uchar>(y,x)>=75 and filter[2].at<uchar>(y,x)<125) + (filter[3].at<uchar>(y,x)>=75 and filter[3].at<uchar>(y,x)<125) 
						+ (filter[4].at<uchar>(y,x)>=75 and filter[4].at<uchar>(y,x)<125))>=NivelFiltro){   
                           posfilter.at<uchar>(y,x) = 170;   
                           //Grava distância do objeto em variavel para monitoração
   
                           if (x<=WIDTH*1/5 and saida[0]<2)   
                               saida[0]=2;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and saida[1]<2)   
                               saida[1]=2;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and saida[2]<2)   
                               saida[2]=2;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and saida[3]<2)   
                               saida[3]=2;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and saida[4]<2)   
                               saida[4]=2;   
  
                           auxpixel = (filter[0].at<uchar>(y,x) + filter[1].at<uchar>(y,x) + filter[2].at<uchar>(y,x) + filter[3].at<uchar>(y,x) + filter[4].at<uchar>(y,x))/5;   
                           //Grava cor do pixel referente à distância do objeto em variavel que será enviada ao controlador fuzzy   
                           if (x<=WIDTH*1/5 and pixel[0]<auxpixel)   
                               pixel[0]=auxpixel;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and pixel[1]<auxpixel)   
                               pixel[1]=auxpixel;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and pixel[2]<auxpixel)   
                               pixel[2]=auxpixel;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and pixel[3]<auxpixel)   
                               pixel[3]=auxpixel;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and pixel[4]<auxpixel)   
                               pixel[4]=auxpixel;   
                       }   
                       //Seleciona objetos muito próximos   
                       else if (((filter[0].at<uchar>(y,x)>=125) + (filter[1].at<uchar>(y,x)>=125) + (filter[2].at<uchar>(y,x)>=125) + (filter[3].at<uchar>(y,x)>=125) 
						+ (filter[4].at<uchar>(y,x)>=125))>=NivelFiltro){   
                           posfilter.at<uchar>(y,x) = 255;   
                           //Grava distância do objeto em variavel para monitoração
   
                           if (x<=WIDTH*1/5 and saida[0]<3)   
                               saida[0]=3;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and saida[1]<3)   
                               saida[1]=3;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and saida[2]<3)   
                               saida[2]=3;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and saida[3]<3)   
                               saida[3]=3;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and saida[4]<3)   
                               saida[4]=3;   
  
                           auxpixel = (filter[0].at<uchar>(y,x) + filter[1].at<uchar>(y,x) + filter[2].at<uchar>(y,x) + filter[3].at<uchar>(y,x) + filter[4].at<uchar>(y,x))/5;   
                           //Grava cor do pixel referente à distância do objeto em variavel que será enviada ao controlador fuzzy   
                           if (x<=WIDTH*1/5 and pixel[0]<auxpixel)   
                               pixel[0]=auxpixel;   
                           else if (x>WIDTH*1/5 and x<=WIDTH*2/5 and pixel[1]<auxpixel)   
                               pixel[1]=auxpixel;   
                           else if (x>WIDTH*2/5 and x<=WIDTH*3/5 and pixel[2]<auxpixel)   
                               pixel[2]=auxpixel;   
                           else if (x>WIDTH*3/5 and x<=WIDTH*4/5 and pixel[3]<auxpixel)   
                               pixel[3]=auxpixel;   
                           else if (x>WIDTH*4/5 and x<=WIDTH*5/5 and pixel[4]<auxpixel)   
                               pixel[4]=auxpixel;   
                       }   
                       else{   
                           posfilter.at<uchar>(y,x) = 0;   
                       }   
                       //Insere linhas para demarcar a imagem, facilitando a monitoração do processo   
                       if (x==WIDTH*1/5 or x==WIDTH*2/5 or x==WIDTH*3/5 or x==WIDTH*4/5){   
                           posfilter.at<uchar>(y,x) = 255;   
                       }   
                   }   
               }   
  
               imshow("PosFiltro", posfilter);   
               //Imprime informações referentes ao conjunto de pertinência e proximidade dos pixels que serão enviadas ao controlador fuzzy   
               printf("%d   %d   %d   %d   %d\n%d %d %d %d %d\n\n", saida[0], saida[1], saida[2], saida[3], saida[4], pixel[0], pixel[1], pixel[2], pixel[3], pixel[4]);   
               //Envia dados ao controlador fuzzy   
               sendto(socket_supervisorio,&pixel[0],sizeof(unsigned char[5]),0,(struct sockaddr*)&endereco_supervisorio,socketsize);    
           }   
       }   
   }   
  
   destroyAllWindows();   
  
   //Encerra câmera   
   Dense3DClose(dense3d);   
   CloseDUOCamera();   
   //Encerra sockets   
   close(socket_supervisorio);   
  
   return 0;   
}