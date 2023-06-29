/*ALGORITHM FOR COMMUNICATION WITH THE HMC5883L ELECTRONIC COMPASS
	The following script establishes communication with the electronic compass. 
Subsequently, it stores the data related to the boat's positioning in shared memory 
addresses. */

/*ALGORITMO PARA COMUNICAÇÃO COM A BÚSSOLA ELETRÔNICA HMC5883L 
	O seguinte script realiza a comunicação com a bússola eletrônica. Em seguida, 
armazena  os  dados  referentes  ao  posicionamento  do  barco  em  endereços  de 
memória compartilhada. */
 
#define _GNU_SOURCE   
#include <stdio.h>   
#include <stdlib.h>   
#define _USE_MATH_DEFINES   
#include <math.h>   
#include <fcntl.h>   
#include <unistd.h>   
#include <string.h>   
#include <sys/ioctl.h>   
#include <sys/types.h>   
#include <sys/stat.h>   
#include <sys/time.h>   
#include <linux/i2c-dev.h>   
#include <sys/ipc.h>   
#include <sys/shm.h>   
  
//Inicialização e configuração do magnetometro   
const int HMC5883L_I2C_ADDR = 0x1E;   
  
#define CONFIG_A 0x00   
#define CONFIG_B 0x01   
#define MODE 0x02   
#define DATA 0x03 //Lê 6 bytes: x msb, x lsb, z msb, z lsb, y msb, y lsb   
#define STATUS 0x09   
#define ID_A 0x0A   
#define ID_B 0x0B   
#define ID_C 0x0C   
#define ID_STRING "H43"   
#define GAIN 1370 //000 setting   
  
//Endereços de memória compartilhada   
#define POSX 9900   
#define KEYFinalizar 9912   
  
void selectDevice(int fd, int addr, char * name)   
{   
    if (ioctl(fd, I2C_SLAVE, addr) < 0)   
    {   
        fprintf(stderr, "%s not present\n", name);   
    }   
}   
  
void writeToDevice(int fd, int reg, int val)   
{   
   char buf[2];   
   buf[0]=reg;   
   buf[1]=val;   
   if (write(fd, buf, 2) != 2){   
       fprintf(stderr, "Can't write to ADXL345\n");   
   }   
}   
   
int main(int argc, char **argv)   
{   
   //Variáveis referentesà leitura de dados do magnetômetro   
   int fd;   
   unsigned char buf[16];   
   struct timeval tv;   
   struct timezone tz;   
   struct timeval data_timestamp;   
   int resolution = 100000;   
   long next_timestamp;   
  
   //Configurações de memória compartilhada   
   int shmidX, shmidFinalizar;   
   char *path="/home/pi/Desktop/Software";   
   int *finalizar;   
   int *sendx;   
   int k;   
  
   if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)   
   {   
       // Open port for reading and writing   
       fprintf(stderr, "Failed to open i2c bus\n");   
  
       return 1;   
   }   
  
   //Inicializa módulo HMC5883L   
   selectDevice(fd, HMC5883L_I2C_ADDR, "HMC5883L");   
  
   //first read the 3 ID bytes   
   buf[0] = ID_A;   
   if ((write(fd, buf, 1)) != 1){   
       // Send the register to read from   
       fprintf(stderr, "Error writing to i2c slave\n");   
   }   
   if (read(fd, buf, 3) != 3) {   
           fprintf(stderr, "Unable to read from HMC5883L\n");   
   }   
   buf[3]=0;   
   printf("Identification: '%s' ",buf);   
   if (strncmp(buf, ID_STRING, 3) == 0)   
       printf("HMC5883L sensor detected\n");   
   else {   
       printf("unknown sensor. Exiting.\n");   
       exit(1);   
   }   
  
   //Envia configurações ao matnetometro   
   //writeToDevice(fd, 0x01, 0);   
   writeToDevice(fd, CONFIG_A, 0b01101000); //8 sample averaging   
   writeToDevice(fd, CONFIG_B, 0b00000000); //max gain   
   writeToDevice(fd, MODE,     0b00000011); //idle mode   
  
   gettimeofday(&data_timestamp,&tz);   
   data_timestamp.tv_sec += 1; //start loggin at start of next second   
   data_timestamp.tv_usec = 0;   
  
   //Inicializa memória compartilhada   
   if((shmidFinalizar=shmget(ftok(path,(key_t)KEYFinalizar),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){   
       printf("\n\nErro na criação do segmento de memoria...\n");   
       exit(-1);   
   }   
  
   finalizar=shmat(shmidFinalizar,0,0);  
   
   if((shmidX=shmget(ftok(path,(key_t)POSX),sizeof(int*),IPC_CREAT|SHM_R|SHM_W))==-1){   
       printf("\n\nErro na criação do segmento de memoria...\n");   
       exit(-1);   
   }   
  
   sendx=shmat(shmidX,0,0);   
  
   //Aguarda comando através de endereço de memória compartilhada para iniciar   
   while ((*finalizar)!=1){   
       usleep(100000);   
       finalizar=shmat(shmidFinalizar,0,0);   
   }   
  
   while ((*finalizar)==1){ //Interrompe a execução quando solicitado por meio de memória compartilhada   
       usleep(200000);//0,2 segundo para iniciar nova leitura   
       while (1) {   
           //get time   
           gettimeofday(&tv,&tz);   
  
           //if seconds >= next_secs && usecs >= next_usecs   
           if (tv.tv_sec >= data_timestamp.tv_sec && tv.tv_usec >= data_timestamp.tv_usec)   
               break;   
  
           usleep(1000);   
       }   
  
       //initiate single conversion   
       writeToDevice(fd, MODE,     0b00000001); //single measurement   
       usleep(7000);   
       buf[0] = DATA;   
  
       //Confirma validade da leitura de dados   
       if ((write(fd, buf, 1)) != 1){   
           fprintf(stderr, "Error writing to i2c slave\n");   
       }   
  
       if (read(fd, buf, 6) != 6){   
           fprintf(stderr, "Unable to read from HMC5883L\n");   
       }   
       else {   
           short x = (buf[0] << 8) | buf[1];   
           short y = (buf[4] << 8) | buf[5];   
           short z = (buf[2] << 8) | buf[3];   
           float angle = atan2(y, x) * 180 / M_PI;   
           float mag = sqrt(x*x+y*y+z*z);   
  
           //Por motivos desconhecidos, o magnetometro passava a decrementar o valor de X quando esse   
           //fosse maior do que 270, enquanto Y presentava valores positivos ao passar por esse ponto.   
           //Dessa forma, corrige-se o posicionamento de x para evitar o problema   
           if (y>0)   
               x=(270-x)+270;   
  
           //Imprime dados lidos do magnetômetro   
           printf("X:%d,   Y:%d,   Z:%d,   %0.1f nT\n",x,y,z,1e5*mag/GAIN);   
           *sendx=x;   
           fflush(stdout);   
       }   
  
       //advance data timestamp to next required time   
       data_timestamp.tv_usec += resolution;   
       if (data_timestamp.tv_usec >= 1e6) {    
           data_timestamp.tv_sec += 1;   
           data_timestamp.tv_usec -= 1e6;   
       }   
  
   }   
  
   //Encerra memória compartilhada   
   shmdt(sendx);   
   shmdt(finalizar);   
  
   return 0;   
}