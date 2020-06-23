#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>

#include "BoatController.h"

using namespace std;
char serial_port[]="/dev/ttyS0";

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

 // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t system_id = 1; // id of computer which is sending the command (ground control software has id of 255, system = 1)
  uint8_t component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t target_component = 0; // Target component, 0 = all (seems to work with 0 or 1)
  int fd;
int main(int argc, char *argv[])
{

   TinyGPSPlus *gps = new TinyGPSPlus();
   char serial_buff[1024];
   char serial_msg[1024];
   int chars_read =0;

   if (gpioInitialise() < 0) return 1;

   gpioSerialReadOpen(18,9600,8);
   fd = serOpen(serial_port, 9600,0);

   sensor_set set;
   set.mag = set.acc = set.gyro = true;

  minimu9::comm_config config = minimu9::auto_detect("/dev/i2c-1");

  minimu9::handle imu;
  imu.open(config);
  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();
  Vec V;
  V.X=0;V.Y=0,V.Z=0;
  std::tuple<float,float,float> acc;
  acc=imu.read_acc();

   while(true)
   {
    chars_read=gpioSerialRead(18,serial_buff,1024);
    if(chars_read>0){
      for(int i=0;i<chars_read;i++)
       {
       gps->encode(serial_buff[i]);
       if(gps->location.isUpdated()){
          sprintf(serial_msg,"LAT=%f, LON=%f, Age:%lld, Heading:%f, Speed:%f\r\n",gps->location.lat(),gps->location.lng(),gps->location.age(),gps->course.deg(),gps->speed.mps());
          printf("%s",serial_msg);
          SendGPS(gps);
          //serWrite(fd,serial_msg,strnlen(serial_msg,1024));
          acc=imu.read_acc();
           float nx= std::get<0>(acc);float ny = std::get<1>(acc);float nz = std::get<2>(acc);
           float roll =atan2(ny, nz)*60;

           float pitch =atan2(nx,nz)*60;
          printf("X:%.2f,Y:%.2f,Z:%.2f, Roll:%.2f,Pitch:%.2f\r\n",nx,ny,nz,roll,pitch);

          }
          
       //printf("%c",serial_buff[i]);
       //serWrite(fd,(char*)&serial_buff[i],1);
       }
      }
   }
   gpioTerminate();
}
int SendGPS(TinyGPSPlus *gps )
{
 uint64_t time_usec = gps->location.time_tag() * 1000;
 uint8_t fix_type = 3;
 int32_t lat = (int32_t)(gps->location.lat()*1e7);
 int32_t lon = (int32_t)(gps->location.lng()*1e7);
 int32_t alt = (int32_t)gps->altitude.meters()*1000;
 uint16_t eph=UINT16_MAX;
 uint16_t epv =UINT16_MAX;
 uint16_t vel = (uint16_t)( gps->speed.mps()*100.0);
 uint16_t cog = (uint16_t)( gps->course.deg()*100.0);
 uint8_t satellites_visible = gps->satellites.value();
 uint8_t dgps_numch = satellites_visible;
 uint32_t dgps_age = (uint32_t)gps->location.age();
 uint16_t yaw =0;

 mavlink_msg_gps2_raw_pack(system_id, component_id, &msg,
                               time_usec, fix_type, lat, lon,  alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw);
 uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

 serWrite(fd,(char*)buf,len);
 mavlink_gps2_raw_t gps2_raw;
 mavlink_msg_gps2_raw_decode(&msg, &gps2_raw);
 printf("Lat:%d, Lon:%d", gps2_raw.lat,gps2_raw.lon );
return len;
}
