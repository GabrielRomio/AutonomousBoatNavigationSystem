/*ALGORITHM FOR DATA EXCHANGE BETWEEN EXTERNAL COMPUTER AND RASPBERRY
	The algorithm expressed below is responsible for receiving and processing 
vectors coming from the external computer through UDP sockets. Every time it 
receives new data, it is transmitted to shared memory addresses so that other 
routines processed on the Raspberry Pi can access it. */

/*ALGORITMO PARA TROCA DE DADOS ENTRE COMPUTADOR EXTERNO E RASPBERRY 
	O algoritmo expresso abaixo é o responsável pelo recebimento e tratamento 
dos vetores provindos do computador externo, por meio de sockets UDP. Cada vez 
que  recebe  um  novo  dado,  o  mesmo  é  transmitido  à  endereços  de  memória 
compartilhada  para  que  as  demais  rotinas  processadas  no  Raspberry  PI  possam 
acessá-lo. */
 
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <arpa/inet.h>     
#include <string.h>     
#include <sys/ipc.h>     
#include <sys/shm.h>     
     
//Endereço para comunicação através de UDP     
#define PORTNOTEBOOK 2302     
    
//Endereços de memória compartilhada     
#define KEYFinalizar 9912     
#define SEND 9920     
    
typedef struct sample{     
       unsigned char sendProcesso[5];     
}sample;     
    
struct dados {     
   float y;     
   int finalizar;     
}__attribute__((__packed__));     
    
int main() {     
   //Variaveis referentes aos sockets     
   socklen_t socketsize; //Recebe o tamanho da estrutura sockaddr_in     
   int socket_notebook;     
   struct sockaddr_in endereco_processo;     
   struct sockaddr_in endereco_notebook;     
    
   unsigned char saida[5];     
    
   //Sockets     
   socket_notebook= socket(AF_INET, SOCK_DGRAM,0);     
   socketsize = sizeof(struct sockaddr_in); //Guarda o tamanho da estrutura sockaddr_in     
   endereco_processo.sin_family = AF_INET;     
   endereco_processo.sin_addr.s_addr = INADDR_ANY; //Utiliza qualquer interface de rede disponivel (Ex: Ethernet, Wireless)     
   endereco_processo.sin_port = htons(PORTNOTEBOOK); //Define a porta de escuta da conexão     
   memset(&(endereco_processo.sin_zero),'\0',sizeof(endereco_processo.sin_zero)); //Zera o resto da estrutura     
   bind(socket_notebook,(struct sockaddr *)&endereco_processo, sizeof(struct sockaddr)); //Liga o endereçamento do servidor ao socket     
    
   //Estabelece comunicação com computador externo      
   endereco_notebook.sin_family = AF_INET;     
   endereco_notebook.sin_addr.s_addr = inet_addr("192.168.1.39"); //IP do computador do qual receberá pacotes     
   endereco_notebook.sin_port = htons(PORTNOTEBOOK); //Define a porta de escuta da conexao     
   memset(&(endereco_notebook.sin_zero),'\0',sizeof(endereco_notebook.sin_zero)); //Zera o resto da estrutura     
    
   saida[0]=0;     
   saida[1]=0;     
   saida[2]=0;     
   saida[3]=0;     
   saida[4]=0;     
    
   //Configurações de memória compartilhada     
   int shmidX, shmidFinalizar;     
   char *path="/home/pi/Desktop/Software";     
   int *finalizar;     
   sample *structDados;     
    
   //Inicializa memória compartilhada     
   if((shmidFinalizar=shmget(ftok(path,(key_t)KEYFinalizar),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){     
       printf("\n\nErro na criação do segmento de memoria...\n");     
       exit(-1);     
   }     
    
   finalizar=shmat(shmidFinalizar,0,0);     
    
   if((shmidX=shmget(ftok(path,(key_t)SEND),sizeof(sample),IPC_CREAT|SHM_R|SHM_W))==-1){     
       printf("\n\nErro na criação do segmento de memoria...\n");     
       exit(-1);     
   }     
    
   structDados=(sample *) shmat(shmidX,0,0);     
    
   //Aguarda comando através de endereço de memória compartilhada para iniciar     
   while ((*finalizar)!=1){     
       usleep(100000);     
       finalizar=shmat(shmidFinalizar,0,0);     
   }     
    
   while((*finalizar)==1) //Interrompe a execução quando solicitado por meio de memória compartilhada     
   {     
       recvfrom(socket_notebook,&saida[0],sizeof(unsigned char[5]),0,(struct sockaddr*)&endereco_notebook,&socketsize);     
       structDados->sendProcesso[0] = saida[0];     
       structDados->sendProcesso[1] = saida[1];     
       structDados->sendProcesso[2] = saida[2];     
       structDados->sendProcesso[3] = saida[3];     
       structDados->sendProcesso[4] = saida[4];     
       printf("Enviou: %d %d %d %d %d\n", structDados->sendProcesso[0], structDados->sendProcesso[1], structDados->sendProcesso[2], structDados->sendProcesso[3], structDados->sendProcesso[4]);     
   }     
    
   //Encerra comunicação com o computador externo     
   close(socket_notebook);     
   //Encerra memória compartilhada     
   shmdt(structDados);     
   shmdt(finalizar);     
    
   return EXIT_SUCCESS;     
}
