/*ALGORITHM FOR PROCESS START AND STOP
	This script only controls the start and stop of other processes, considering 
numerical commands from a keyboard as input. */

/*ALGORITMO PARA INICIAR E FINALIZAR PROCESSO 
	Este script realiza apenas o controle de início e parada dos demais processos, 
considerando comandos numéricos de um teclado como entrada. */
 
#include <stdio.h>     
#include <stdlib.h>     
#include <unistd.h>     
#include <arpa/inet.h>     
#include <string.h>     
#include <sys/ipc.h>     
#include <sys/shm.h>     
     
//Endereços de memória compartilhada     
#define KEYFinalizar 9912     
    
int main() {     
   //Configurações de memória compartilhada     
   int shmidFinalizar;     
   char *path="/home/pi/Desktop/Software";     
   int *finalizar;     
   int auxEntrada=0;     
    
   //Inicializa memória compartilhada     
   if((shmidFinalizar=shmget(ftok(path,(key_t)KEYFinalizar),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){     
       printf("\n\nErro na criação do segmento de memoria...\n");     
       exit(-1);     
   }     
    
   finalizar=shmat(shmidFinalizar,0,0);     
    
   //Aguarda comando de teclado para inicializar execução     
   while (auxEntrada!=1){     
       usleep(100000);     
       printf("Digite 1 para iniciar:");     
       scanf("%i", &auxEntrada);     
   }     
    
   //Seta endereço de memória compartilhada para iniciar execução de todos os programas     
   *finalizar=1;     
    
   //Aguarda comando de teclado para finalizar programa     
   while(auxEntrada!=2){     
       usleep(100000);     
       printf("Digite 2 para sair:");     
       scanf("%i", &auxEntrada);     
   }     
    
   //Encerra endereço de memória compartilhada interrompendo a execução de todos os programas     
   *finalizar=0;     
   shmdt(finalizar);     
   if((shmctl(shmidFinalizar,IPC_RMID,NULL))==-1){     
       printf("Erro ao destruir o segmento.\n");     
       exit(-1);     
   }     
    
   return EXIT_SUCCESS;     
}  
