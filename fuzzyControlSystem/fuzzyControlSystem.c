/*ALGORITHM FOR SYSTEM CONTROL BASED ON FUZZY LOGIC
	The following algorithm reads all data from the set through shared memory addresses and 
applies them to a fuzzy controller. It performs processes of fuzzification, application 
of fuzzy rules, and defuzzification of the obtained values. Subsequently, it uses these 
results to command the vehicle's navigation, controlling the activation of its two motors 
through digital outputs. */

/*ALGORITMO PARA CONTROLE DO SISTEMA BASEADO EM LÓGICA FUZZY 
	O seguinte algoritmo realiza a leitura de todos os dados do conjunto, por meio 
de endereços de memória compartilhada, e os aplica em um controlador fuzzy. Assim, 
desempenha  processos  de  fuzzificação,  aplicação  de  regras  nebulosas  e 
defuzzificação  dos  valores  obtidos.  Na  sequência,  utiliza  esses  resultados  para 
comandar a navegação do veículo, controlando o acionamento dos seus dois motores 
por meio de saídas digitais. */
 
#include <stdio.h>   
#include <math.h>   
#include <stdlib.h>   
#include <sys/ipc.h>   
#include <sys/shm.h>   
#include <unistd.h>   
#include <sys/time.h>   
#include <wiringPi.h>   
#include <arpa/inet.h>   
#include <string.h>   
  
//Endereços de memória compartilhada   
#define POSMagnet 9900   
#define KEYFinalizar 9912   
#define KEYSAIDA 9920   
  
//Estruturas para recebimento de dados de endereços de memória compartilhada   
typedef struct sample{   
   unsigned char entrada[5];   
}sample;   
  
struct dados {   
   float y;   
   int finalizar;   
}__attribute__((__packed__));   
  
//Estruturas referentes aos conjuntos utilizados no processo de fuzzificação   
typedef struct StructDistancia {   
   float Livre, Distante, DistanciaMedia, Proximo;   
}StructDistancia;   
  
typedef struct StructPosicaoAngular {   
   float Esquerda, EsquerdaLeve, Centro, DireitaLeve, Direita;   
}StructPosicaoAngular;   
  
int main()   
{   
   //Variaveis referentes à memória compartilhada   
   int shmidMagnet;   
   int shmidFinalizar;   
   int shmidSaida;   
   char *path="/home/pi/Desktop/Software";   
   int *finalizar;   
   int *magnet;   
   sample *saida;   
   
   //Declaração das demais variáveis   
   int VariacaoAngular;   
   int PosicaoAngularDesejada = 240; //Define a posição angular desejada   
   int AuxMaiorValor;   
   float Esquerda, EsquerdaLeve, Frente, DireitaLeve, Direita, Re;   
   float AuxEsquerda1, AuxEsquerda2, AuxEsquerda3, AuxEsquerda4, AuxEsquerdaLeve1, AuxEsquerdaLeve2, AuxEsquerdaLeve3;   
   float AuxDireitaLeve1, AuxDireitaLeve2, AuxDireitaLeve3, AuxDireita1, AuxDireita2, AuxDireita3, AuxDireita4;   
   float SaidaMOM;   
  
   //Definição das estruturas para variáveis fuzzy   
   StructDistancia u0; //Obstáculos à esquerda   
   StructDistancia u1; //Obstáculos ao centro-esquerda   
   StructDistancia u2; //Obstáculos ao centro   
   StructDistancia u3; //Obstáculos ao centro-direita   
   StructDistancia u4; //Obstáculos à direita   
  
   StructPosicaoAngular PosicaoAngular; //Posição angular da inclinação do barco   
  
   wiringPiSetup();        //Inicia a biblioteca WiringPi, responsável pelas entradas e saídas do Raspberry   
   pinMode(1, PWM_OUTPUT); //Configura o pino físico 12 como saída PWM dos motores -- NÃO UTILIZADO   
   pinMode(0, OUTPUT); //Configura o pino físico 11 como saída IN1   
   pinMode(2, OUTPUT); //Configura o pino físico 13 como saída IN2   
   pinMode(3, OUTPUT); //Configura o pino físico 15 como saída IN3   
   pinMode(4, OUTPUT); //Configura o pino físico 16 como saída IN4   
  
   int estado=0;   
  
   //Inicializa memória compartilhada   
   if((shmidMagnet=shmget(ftok(path,(key_t)POSMagnet),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){   
       printf("\n\nErro na criação do segmento de memoria...\n");   
       exit(-1);   
   }   
  
   if((shmidSaida=shmget(ftok(path,(key_t)KEYSAIDA),sizeof(sample),IPC_CREAT|SHM_R|SHM_W))==-1){   
       printf("\n\nErro na criação do segmento de memoria...\n");   
       exit(-1);   
   }   
  
   if((shmidFinalizar=shmget(ftok(path,(key_t)KEYFinalizar),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){   
       printf("\n\nErro na criação do segmento de memoria...\n");   
       exit(-1);   
   }   
  
   magnet = shmat(shmidMagnet, 0, 0);   
   finalizar = shmat(shmidFinalizar, 0, 0);   
   saida = (sample *) shmat(shmidSaida,0,0);   
  
   //Aguarda comando através de endereço de memória compartilhada para iniciar   
   while ((*finalizar)!=1){   
       usleep(100000);   
       finalizar=shmat(shmidFinalizar,0,0);   
   }   
  
   while((*finalizar)==1) //Interrompe a execução quando solicitado por meio de memória compartilhada   
   {   
       //Fuzzificação das variáveis de entrada   
  
       //u0    
       if (saida->entrada[0]<=40)   
           u0.Livre = 1;   
       else if (saida->entrada[0]<=60)   
           u0.Livre = (60.0-saida->entrada[0])/20.0;   
       else   
           u0.Livre = 0;   
  
       if (saida->entrada[0]<=40)   
           u0.Distante = 0;   
       else if (saida->entrada[0]<=60)   
           u0.Distante = (saida->entrada[0]-40.0)/20.0;   
       else if (saida->entrada[0]<=65)   
           u0.Distante = 1;   
       else if (saida->entrada[0]<=85)   
           u0.Distante = (85.0-saida->entrada[0])/20.0;   
       else   
           u0.Distante = 0;   
  
       if (saida->entrada[0]<=65)   
           u0.DistanciaMedia = 0;   
       else if (saida->entrada[0]<=85)   
           u0.DistanciaMedia = (saida->entrada[0]-65.0)/20.0;   
       else if (saida->entrada[0]<=115)   
           u0.DistanciaMedia = 1;   
       else if (saida->entrada[0]<=135)   
           u0.DistanciaMedia = (135.0-saida->entrada[0])/20.0;   
       else   
           u0.DistanciaMedia = 0;   
  
       if (saida->entrada[0]<=115)   
           u0.Proximo = 0;   
       else if (saida->entrada[0]<=135)   
           u0.Proximo = (saida->entrada[0]-115.0)/20.0;   
       else   
           u0.Proximo = 1;   
  
       //u1   
       if (saida->entrada[1]<=40)   
           u1.Livre = 1;   
       else if (saida->entrada[1]<=60)   
           u1.Livre = (60.0-saida->entrada[1])/20.0;   
       else   
           u1.Livre = 0;   
  
       if (saida->entrada[1]<=40)   
           u1.Distante = 0;   
       else if (saida->entrada[1]<=60)   
           u1.Distante = (saida->entrada[1]-40.0)/20.0;   
       else if (saida->entrada[1]<=65)   
           u1.Distante = 1;   
       else if (saida->entrada[1]<=85)   
           u1.Distante = (85.0-saida->entrada[1])/20.0;   
       else   
           u1.Distante = 0;   
  
       if (saida->entrada[1]<=65)   
           u1.DistanciaMedia = 0;   
       else if (saida->entrada[1]<=85)   
           u1.DistanciaMedia = (saida->entrada[1]-65.0)/20.0;   
       else if (saida->entrada[1]<=115)   
           u1.DistanciaMedia = 1;   
       else if (saida->entrada[1]<=135)   
           u1.DistanciaMedia = (135.0-saida->entrada[1])/20.0;   
       else   
           u1.DistanciaMedia = 0;   
   
       if (saida->entrada[1]<=115)   
           u1.Proximo = 0;   
       else if (saida->entrada[1]<=135)   
           u1.Proximo = (saida->entrada[1]-115.0)/20.0;   
       else   
           u1.Proximo = 1;   
  
       //u2   
       if (saida->entrada[2]<=40)   
           u2.Livre = 1;   
       else if (saida->entrada[2]<=60)   
           u2.Livre = (60.0-saida->entrada[2])/20.0;   
       else   
           u2.Livre = 0;   
  
       if (saida->entrada[2]<=40)   
           u2.Distante = 0;   
       else if (saida->entrada[2]<=60)   
           u2.Distante = (saida->entrada[2]-40.0)/20.0;   
       else if (saida->entrada[2]<=65)   
           u2.Distante = 1;   
       else if (saida->entrada[2]<=85)   
           u2.Distante = (85.0-saida->entrada[2])/20.0;   
       else   
           u2.Distante = 0;   
  
       if (saida->entrada[2]<=65)   
           u2.DistanciaMedia = 0;   
       else if (saida->entrada[2]<=85)   
           u2.DistanciaMedia = (saida->entrada[2]-65.0)/20.0;   
       else if (saida->entrada[2]<=115)   
           u2.DistanciaMedia = 1;   
       else if (saida->entrada[2]<=135)   
           u2.DistanciaMedia = (135.0-saida->entrada[2])/20.0;   
       else   
           u2.DistanciaMedia = 0;   
  
       if (saida->entrada[2]<=115)   
           u2.Proximo = 0;   
       else if (saida->entrada[2]<=135)   
           u2.Proximo = (saida->entrada[2]-115.0)/20.0;   
       else   
           u2.Proximo = 1;   
  
       //u3   
       if (saida->entrada[3]<=40)   
           u3.Livre = 1;   
       else if (saida->entrada[3]<=60)   
           u3.Livre = (60.0-saida->entrada[3])/20.0;   
       else   
           u3.Livre = 0;   
  
       if (saida->entrada[3]<=40)   
           u3.Distante = 0;   
       else if (saida->entrada[3]<=60)   
           u3.Distante = (saida->entrada[3]-40.0)/20.0;   
       else if (saida->entrada[3]<=65)   
           u3.Distante = 1;   
       else if (saida->entrada[3]<=85)   
           u3.Distante = (85.0-saida->entrada[3])/20.0;   
       else   
           u3.Distante = 0;   
  
       if (saida->entrada[3]<=65)   
           u3.DistanciaMedia = 0;   
       else if (saida->entrada[3]<=85)    
           u3.DistanciaMedia = (saida->entrada[3]-65.0)/20.0;   
       else if (saida->entrada[3]<=115)   
           u3.DistanciaMedia = 1;   
       else if (saida->entrada[3]<=135)   
           u3.DistanciaMedia = (135.0-saida->entrada[3])/20.0;   
       else   
           u3.DistanciaMedia = 0;   
  
       if (saida->entrada[3]<=115)   
           u3.Proximo = 0;   
       else if (saida->entrada[3]<=135)   
           u3.Proximo = (saida->entrada[3]-115.0)/20.0;   
       else   
           u3.Proximo = 1;   
  
       //u4   
       if (saida->entrada[4]<=40)   
           u4.Livre = 1;   
       else if (saida->entrada[4]<=60)   
           u4.Livre = (60.0-saida->entrada[4])/20.0;   
       else   
           u4.Livre = 0;   
  
       if (saida->entrada[4]<=40)   
           u4.Distante = 0;   
       else if (saida->entrada[4]<=60)   
           u4.Distante = (saida->entrada[4]-40.0)/20.0;   
       else if (saida->entrada[4]<=65)   
           u4.Distante = 1;   
       else if (saida->entrada[4]<=85)   
           u4.Distante = (85.0-saida->entrada[4])/20.0;   
       else   
           u4.Distante = 0;   
  
       if (saida->entrada[4]<=65)   
           u4.DistanciaMedia = 0;   
       else if (saida->entrada[4]<=85)   
           u4.DistanciaMedia = (saida->entrada[4]-65.0)/20.0;   
       else if (saida->entrada[4]<=115)   
           u4.DistanciaMedia = 1;   
       else if (saida->entrada[4]<=135)   
           u4.DistanciaMedia = (135.0-saida->entrada[4])/20.0;   
       else   
           u4.DistanciaMedia = 0;   
  
       if (saida->entrada[4]<=115)   
           u4.Proximo = 0;   
       else if (saida->entrada[4]<=135)   
           u4.Proximo = (saida->entrada[4]-115.0)/20.0;   
       else   
           u4.Proximo = 1;   
  
       //Variação da posição angular   
       VariacaoAngular = -*magnet + PosicaoAngularDesejada;   
  
       if (VariacaoAngular<=-60)   
           PosicaoAngular.Esquerda = 1;   
       else if (VariacaoAngular<=-30)   
          PosicaoAngular.Esquerda = (-VariacaoAngular-30.0)/30.0;   
       else   
           PosicaoAngular.Esquerda = 0;   
  
       if (VariacaoAngular<=-60)   
           PosicaoAngular.EsquerdaLeve = 0;   
       else if (VariacaoAngular<=-30)   
           PosicaoAngular.EsquerdaLeve = (60.0+VariacaoAngular)/30.0;   
       else if (VariacaoAngular<=-10)   
           PosicaoAngular.EsquerdaLeve = 1;   
       else if (VariacaoAngular<=0)   
           PosicaoAngular.EsquerdaLeve = (-VariacaoAngular)/10.0;   
       else   
          PosicaoAngular.EsquerdaLeve = 0;   
  
       if (VariacaoAngular<=-10)   
           PosicaoAngular.Centro = 0;   
       else if (VariacaoAngular<=0)   
           PosicaoAngular.Centro = (10.0+VariacaoAngular)/10.0;   
       else if (VariacaoAngular<=10)   
           PosicaoAngular.Centro = (10.0-VariacaoAngular)/10.0;   
       else   
          PosicaoAngular.Centro = 0;   
  
       if (VariacaoAngular<=0)   
           PosicaoAngular.DireitaLeve = 0;   
       else if (VariacaoAngular<=10)   
           PosicaoAngular.DireitaLeve = (VariacaoAngular)/10.0;   
       else if (VariacaoAngular<=30)   
           PosicaoAngular.DireitaLeve = 1;   
       else if (VariacaoAngular<=60)   
           PosicaoAngular.DireitaLeve = (60.0-VariacaoAngular)/30.0;   
       else   
          PosicaoAngular.DireitaLeve = 0;   
  
       if (VariacaoAngular<=30)   
           PosicaoAngular.Direita = 0;   
       else if (VariacaoAngular<=60)   
          PosicaoAngular.Direita = (VariacaoAngular-30.0)/30.0;   
       else   
           PosicaoAngular.Direita = 1;   
  
       //Regras fuzzy   
  
       //n°1 - SE (max(u3, u4)=DISTANTE E max(u3, u4)=LIVRE) ENTAO DIREITA=(VPA->ESQUERDA)   
       if (((u3.Distante>=0.5) || (u3.Livre>=0.5)) && ((u4.Distante>=0.5) || (u4.Livre>=0.5)))   
           AuxDireita1 = PosicaoAngular.Esquerda;   
       else   
           AuxDireita1 = 0;   
  
       //n°2 - SE (max(u3, u4)=DISTANTE E max(u3, u4)=LIVRE) ENTAO DIREITA LEVE=(VPA->ESQUERDA LEVE)   
       if (((u3.Distante>=0.5) || (u3.Livre>=0.5)) && ((u4.Distante>=0.5) || (u4.Livre>=0.5)))   
           AuxDireitaLeve1 = PosicaoAngular.EsquerdaLeve;   
       else   
           AuxDireitaLeve1 = 0;   
  
       //n°3 - SE (max(u0, u1)=DISTANTE E max(u0, u1)=LIVRE) ENTAO ESQUERDA=(VPA->DIREITA)   
       if (((u0.Distante>=0.5) || (u0.Livre>=0.5)) && ((u1.Distante>=0.5) || (u1.Livre>=0.5)))   
           AuxEsquerda1 = PosicaoAngular.Direita;   
       else   
           AuxEsquerda1 = 0;   
  
       //n°4 - SE (max(u0, u1)=DISTANTE E max(u0, u1)=LIVRE) ENTAO ESQUERDA LEVE=(VPA->DIREITA LEVE)   
       if (((u0.Distante>=0.5) || (u0.Livre>=0.5)) && ((u1.Distante>=0.5) || (u1.Livre>=0.5)))   
           AuxEsquerdaLeve1 = PosicaoAngular.DireitaLeve;   
       else    
           AuxEsquerdaLeve1 = 0;   
  
       //n°5 - SE (VPA=CENTRO) E (u2=LIVRE) E u1=(DISTANTE OU LIVRE) E u3=(DISTANTE OU LIVRE) ENTAO FRENTE=(u2->LIVRE)   
       if ((PosicaoAngular.Centro>=0.5) && (u2.Livre>=0.5) && ((u1.Distante>=0.5) || (u1.Livre>=0.5)) && ((u3.Distante>=0.5) || (u3.Livre>=0.5)))   
           Frente = u2.Livre;   
       else   
           Frente = 0;   
  
       //n°6 - SE max(u0,u1)<=max(u3,u4) E (max(u3,u4)=DISTANTE) ENTAO ESQUERDA LEVE=(u3->DISTANTE OU u4->DISTANTE)   
       if (saida->entrada[3]>=saida->entrada[4])   
           AuxMaiorValor = saida->entrada[3];   
       else   
           AuxMaiorValor = saida->entrada[4];   
  
       if ((saida->entrada[0]<=AuxMaiorValor) && (saida->entrada[1]<=AuxMaiorValor) && ((u3.Distante>=0.5) || (u4.Distante>=0.5)))   
           if (u3.Distante>u4.Distante)   
               AuxEsquerdaLeve2 = u3.Distante;   
           else   
               AuxEsquerdaLeve2 = u4.Distante;   
       else   
           AuxEsquerdaLeve2 = 0;   
  
       //n°7 - SE max(u0,u1)<=max(u3,u4) E (max(u3,u4)=DISTANCIA MEDIA) ENTAO ESQUERDA=(u3->DISTANCIA MEDIA OU u4->DISTANCIA MEDIA)   
       if ((saida->entrada[0]<=AuxMaiorValor) && (saida->entrada[1]<=AuxMaiorValor) && ((u3.DistanciaMedia>=0.5) || (u4.DistanciaMedia>=0.5)))   
           if (u3.DistanciaMedia>u4.DistanciaMedia)   
               AuxEsquerda2 = u3.DistanciaMedia;   
           else   
               AuxEsquerda2 = u4.DistanciaMedia;   
       else   
           AuxEsquerda2 = 0;   
  
       //n°8 - SE max(u0,u1)<=max(u3,u4) E (max(u3,u4)=PROXIMO) ENTAO ESQUERDA=1   
       if ((saida->entrada[0]<=AuxMaiorValor) && (saida->entrada[1]<=AuxMaiorValor) && ((u3.Proximo>=0.5) || (u4.Proximo>=0.5)))   
           AuxEsquerda3 = 1;   
       else   
           AuxEsquerda3 = 0;   
  
       //n°9 - SE max(u0,u1)<=max(u3,u4) E (u2=DISTANTE) ENTAO ESQUERDA LEVE=(u2->DISTANTE)   
       if ((saida->entrada[0]<=AuxMaiorValor) && (saida->entrada[1]<=AuxMaiorValor) && (u2.Distante>=0.5))   
           AuxEsquerdaLeve3 = u2.Distante;   
       else   
           AuxEsquerdaLeve3 = 0;   
  
       //n°10 - SE max(u0,u1)<=max(u3,u4) E (u2=DISTANCIA MEDIA) ENTAO ESQUERDA=(u2->DISTANCIA MEDIA)   
       if ((saida->entrada[0]<=AuxMaiorValor) && (saida->entrada[1]<=AuxMaiorValor) && (u2.DistanciaMedia>=0.5))   
           AuxEsquerda4 = u2.DistanciaMedia;   
       else   
           AuxEsquerda4 = 0;   
  
       //n°11 - SE max(u3,u4)<=max(u0,u1) E (max(u0,u1)=DISTANTE) ENTAO DIREITA LEVE=(u0->DISTANTE OU u1->DISTANTE)   
       if (saida->entrada[0]>=saida->entrada[1])   
           AuxMaiorValor = saida->entrada[0];   
       else    
           AuxMaiorValor = saida->entrada[1];   
  
       if ((saida->entrada[3]<=AuxMaiorValor) && (saida->entrada[4]<=AuxMaiorValor) && ((u0.Distante>=0.5) || (u1.Distante>=0.5)))   
           if (u0.Distante>u1.Distante)   
               AuxDireitaLeve2 = u0.Distante;   
           else   
               AuxDireitaLeve2 = u1.Distante;   
       else   
           AuxDireitaLeve2 = 0;   
  
       //n°12 - SE max(u3,u4)<=max(u0,u1) E (max(u0,u1)=DISTANCIA MEDIA) ENTAO DIREITA=(u0->DISTANCIA MEDIA OU u1->DISTANCIA MEDIA)   
       if ((saida->entrada[3]<=AuxMaiorValor) && (saida->entrada[4]<=AuxMaiorValor) && ((u0.DistanciaMedia>=0.5) || (u1.DistanciaMedia>=0.5)))   
           if (u0.DistanciaMedia>u1.DistanciaMedia)   
               AuxDireita2 = u0.DistanciaMedia;   
           else   
               AuxDireita2 = u1.DistanciaMedia;   
       else   
           AuxDireita2 = 0;   
  
       //n°13 - SE max(u3,u4)<=max(u0,u1) E (max(u0,u1)=PROXIMO) ENTAO DIREITA=1   
       if ((saida->entrada[3]<=AuxMaiorValor) && (saida->entrada[4]<=AuxMaiorValor) && ((u0.Proximo>=0.5) || (u1.Proximo>=0.5)))   
           AuxDireita3 = 1;   
       else   
           AuxDireita3 = 0;   
  
       //n°14 - SE max(u3,u4)<=max(u0,u1) E (u2=DISTANTE) ENTAO DIREITA LEVE=(u2->DISTANTE)   
       if ((saida->entrada[3]<=AuxMaiorValor) && (saida->entrada[4]<=AuxMaiorValor) && (u2.Distante>=0.5))   
           AuxDireitaLeve3 = u2.Distante;   
       else   
           AuxDireitaLeve3 = 0;   
  
       //n°15 - SE max(u3,u4)<=max(u0,u1) E (u2=DISTANCIA MEDIA) ENTAO DIREITA=(u2->DISTANCIA MEDIA)   
       if ((saida->entrada[3]<=AuxMaiorValor) && (saida->entrada[4]<=AuxMaiorValor) && (u2.DistanciaMedia>=0.5))   
           AuxDireita4 = u2.DistanciaMedia;   
       else   
           AuxDireita4 = 0;   
  
       //n°16 - SE (u2=PROXIMO) OU (max(u0,u1)=PROXIMO E max(u3,u4)=PROXIMO) ENTÃO RÉ=1   
       if ((u2.Proximo>=0.5) || (((u0.Proximo>=0.5) || (u1.Proximo>=0.5)) && ((u3.Proximo>=0.5) || (u4.Proximo>=0.5))))   
           Re = 1;   
       else   
           Re = 0;   
  
       //Considera o maior valor de cada conjunto para utilizá-lo no processo de defuzzificação   
       if ((AuxEsquerdaLeve1 >= AuxEsquerdaLeve2) && (AuxEsquerdaLeve1 >= AuxEsquerdaLeve3))   
           EsquerdaLeve = AuxEsquerdaLeve1;   
       else if (AuxEsquerda2 >= AuxEsquerdaLeve3)   
           EsquerdaLeve = AuxEsquerdaLeve2;   
       else   
           EsquerdaLeve = AuxEsquerdaLeve3;   
  
       if ((AuxEsquerda1 >= AuxEsquerda2) && (AuxEsquerda1 >= AuxEsquerda3) && (AuxEsquerda1 >= AuxEsquerda4))    
           Esquerda = AuxEsquerda1;   
       else if ((AuxEsquerda2 >= AuxEsquerda3) && (AuxEsquerda2 >= AuxEsquerda4))   
           Esquerda = AuxEsquerda2;   
       else if (AuxEsquerda3 >= AuxEsquerda4)   
           Esquerda = AuxEsquerda3;   
       else   
           Esquerda = AuxEsquerda4;   
  
       if ((AuxDireitaLeve1 >= AuxDireitaLeve2) && (AuxDireitaLeve1 >= AuxDireitaLeve3))   
           DireitaLeve = AuxDireitaLeve1;   
       else if (AuxDireitaLeve2 >= AuxDireitaLeve3)   
           DireitaLeve = AuxDireitaLeve2;   
       else   
           DireitaLeve = AuxDireitaLeve3;   
  
       if ((AuxDireita1 >= AuxDireita2) && (AuxDireita1 >= AuxDireita3) && (AuxDireita1 >= AuxDireita4))   
           Direita = AuxDireita1;   
       else if ((AuxDireita2 >= AuxDireita3) && (AuxDireita2 >= AuxDireita4))  
           Direita = AuxDireita2;   
       else if (AuxDireita3 >= AuxDireita4)   
           Direita = AuxDireita3;   
       else   
           Direita = AuxDireita4;   
  
       //Defuzificação   
  
       //Cálculo da saída através da média do máximo MOM   
       SaidaMOM = (5*Esquerda+25*EsquerdaLeve+45*Frente+65*DireitaLeve+85*Direita)/(Esquerda+EsquerdaLeve+Frente+DireitaLeve+Direita);   
  
       //Seleciona a saída com base no resultado obtido   
       if (Re==1) //Ré   
           estado = 6;   
       else if (SaidaMOM<=15) //Esquerda   
           estado = 1;   
       else if (SaidaMOM<=35) //Esquerda leve   
           estado = 2;   
       else if (SaidaMOM<=55) //Frente   
           estado = 3;   
       else if (SaidaMOM<=75) //Direita leve   
           estado = 4;   
       else //Direita   
           estado = 5;   
  
       //Estados referentes à saída   
  
       switch (estado)   
       {   
           case 0: //Parado   
           printf("Parado\n");   
               pwmWrite(1, 0);   
               digitalWrite(0, LOW);   
               digitalWrite(2, LOW);   
               digitalWrite(3, LOW);   
               digitalWrite(4, LOW);   
           break;   
  
           case 1: //Esquerda   
           printf("Esquerda\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, LOW);   
               digitalWrite(2, HIGH);   
               digitalWrite(3, HIGH);   
               digitalWrite(4, LOW);   
           break;   
  
           case 2: //Esquerda leve   
           printf("Esquerda leve\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, LOW);   
               digitalWrite(2, HIGH);   
               digitalWrite(3, LOW);   
               digitalWrite(4, LOW);   
           break;   
  
           case 3: //Frente   
           printf("Frente\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, LOW);   
               digitalWrite(2, HIGH);   
               digitalWrite(3, LOW);   
               digitalWrite(4, HIGH);   
           break;   
  
           case 4: //Direita leve   
           printf("Direita leve\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, LOW);   
               digitalWrite(2, LOW);   
               digitalWrite(3, LOW);   
               digitalWrite(4, HIGH);   
           break;   
  
           case 5: //Direita   
           printf("Direita\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, HIGH);   
               digitalWrite(2, LOW);   
               digitalWrite(3, LOW);   
               digitalWrite(4, HIGH);   
           break;   
  
           case 6: //Ré   
           printf("Re\n");   
               pwmWrite(1, 1024);   
               digitalWrite(0, HIGH);   
               digitalWrite(2, LOW);   
               digitalWrite(3, HIGH);   
               digitalWrite(4, LOW);   
           usleep(2000000); //Ré dura ao menos 2s, para evitar possíveis loops de movimento   
           break;   
  
           case 7: //Monitora dados recebidos   
           printf("x = %d\n",*magnet);   
           //recvfrom(socket_supervisorio,&saida[0],sizeof(char[5]),0,(struct sockaddr*)&endereco_supervisorio,&socketsize);   
           printf("Recebeu: %d %d %d %d %d\n", saida->entrada[0], saida->entrada[1], saida->entrada[2], saida->entrada[3], saida->entrada[4]);   
           //printf("Recebeu: %d\n", *saida);   
           break;   
  
       }   
   }   
  
   //Reset das saídas digitais ao encerrar programa   
   pwmWrite(1, 0);   
   digitalWrite(0, LOW);   
   digitalWrite(2, LOW);   
   digitalWrite(3, LOW); 
   digitalWrite(4, LOW);   
  
   //Encerra memória compartilhada e comunicação wireless   
   shmdt(magnet);   
   shmdt(saida);   
   shmdt(finalizar);   
   if((shmctl(shmidMagnet,IPC_RMID,NULL))==-1){   
       printf("Erro ao destruir o segmento.\n");   
       exit(-1);   
   }   
   if((shmctl(shmidSaida,IPC_RMID,NULL))==-1){   
       printf("Erro ao destruir o segmento.\n");   
       exit(-1);   
   }   
  
   return EXIT_SUCCESS;   
} 