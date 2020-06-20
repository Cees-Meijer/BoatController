#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>

#include <stdio.h>
#include <pigpio.h>
#include <TinyGPS.h>

using namespace std;
char serial_port[]="/dev/ttyS0";

int main(int argc, char *argv[])
{

   TinyGPSPlus *gps = new TinyGPSPlus();
   char serial_buff[1024];
   char serial_msg[1024];
   int chars_read =0;
   int fd;
   if (gpioInitialise() < 0) return 1;

   gpioSerialReadOpen(18,9600,8);
   fd = serOpen(serial_port, 9600,0);
   while(true)
   {
    chars_read=gpioSerialRead(18,serial_buff,1024);
    if(chars_read>0){
      for(int i=0;i<chars_read;i++)
       {
       gps->encode(serial_buff[i]);
       if(gps->location.isUpdated()){
          sprintf(serial_msg,"LAT=%f, LON=%f, Age:%d, Heading:%f, Speed:%f\r\n",gps->location.lat(),gps->location.lng(),gps->location.age(),gps->course.deg(),gps->speed.mps());
          printf("%s",serial_msg);
          serWrite(fd,serial_msg,strnlen(serial_msg,1024));
          }

       //printf("%c",serial_buff[i]);
       //serWrite(fd,(char*)&serial_buff[i],1);
       }
      }
   }
   gpioTerminate();
}
