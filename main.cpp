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
 // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000);
  auto start = std::chrono::steady_clock::now();
   printf("Controller started. Waiting for GPS fix...\r\n");
  auto heartbeat_timer = start;
  int MAVByteCount=0;
   while(true)
   {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9; 
    duration =  start-heartbeat_timer ;
    if(duration.count()>1e9){
       SendHeartBeat();
       heartbeat_timer = start;
       }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();
    imu.fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);
    float nx= acceleration(0);float ny = acceleration(1);float nz = acceleration(2);
    float roll =atan2(ny, nz)*60;
    float pitch =atan2(nx,nz)*60;
    matrix m = rotation.toRotationMatrix();
    float northx = m(0,0);
    float eastx =  m(1,0);
    float heading = atan2(northx,eastx)*(180 / M_PI) - 90;
    if (heading < 0) heading += 360;
    //printf("LAT=%f, LON=%f, Age:%05lld, Roll:%.2f,Pitch:%.2f,Heading:%.2f\r",gps->location.lat(),gps->location.lng(),gps->location.age(),roll,pitch,heading);

    chars_read=gpioSerialRead(18,serial_buff,1024);
    if(chars_read>0){
      for(int i=0;i<chars_read;i++)
       {
       gps->encode(serial_buff[i]);
       if(gps->location.isUpdated()){
         // sprintf(serial_msg,"LAT=%f, LON=%f, Age:%lld, Heading:%f, Speed:%f\r\n",gps->location.lat(),gps->location.lng(),gps->location.age(),gps->course.deg(),gps->speed.mps());
         // printf("%s",serial_msg);
          SendGPS(gps,heading);
          //serWrite(fd,serial_msg,strnlen(serial_msg,1024));
           
          }

       //printf("%c",serial_buff[i]);
       serWrite(fd,(char*)&serial_buff[i],1);
       }
      }
      
   chars_read = serDataAvailable(fd);
   if (chars_read>0){
      serRead(fd,serial_buff,chars_read);
      for(int i=0;i<chars_read;i++)
       { if(serial_buff[i] == 0xFD){printf("\r\n");MAVByteCount =0;}
          printf("%02X ",serial_buff[i]); 
          if(MAVByteCount == 7 && serial_buff[i]==0x15)
          {
             SendParams();
             printf("Sending Params\r\n");
          }
          MAVByteCount++;
          }
       
       fflush(stdout);
      }
    try
     {  
     loop_pacer.pace();
     }
     catch(std::exception &e){printf("Error in ' pace' \r\n");}
   }
   gpioTerminate();
}

int SendParams()
{
   char param_id[]="PARAM1";
   float param_value = 1.2;
    uint8_t param_type = 0;
     uint16_t param_count =1;
     uint16_t param_index =0;
    
   mavlink_msg_param_value_pack(system_id,component_id, &msg,
                               param_id, param_value, param_type, param_count, param_index);
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

   serWrite(fd,(char*)buf,len);
   return len;
                               
}

// MAVLink Heartbeat ID=0. Len = 21 Bytes.
int SendHeartBeat()
{
   uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
   uint32_t custom_mode =0;
   uint8_t system_status =	MAV_STATE_STANDBY;
   mavlink_msg_heartbeat_pack( system_id, component_id, &msg,
                               	MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, base_mode, custom_mode, system_status);
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

   serWrite(fd,(char*)buf,len);
   return len;
   }

int SendGPS(TinyGPSPlus *gps,float heading )
{
  
 uint64_t time_usec = gps->location.time_tag() * 1000;
 uint8_t fix_type = 3;
 int32_t lat = (int32_t)(gps->location.lat()*1e7);
 int32_t lon = (int32_t)(gps->location.lng()*1e7);
 int32_t alt = (int32_t)gps->altitude.meters()*1000;
 int32_t alt_ellipsoid = alt; ///TEST
 uint16_t eph=UINT16_MAX;
 uint16_t epv =UINT16_MAX;
 uint16_t vel = (uint16_t)( gps->speed.mps()*100.0);
 uint16_t cog = (uint16_t)( gps->course.deg()*100.0);
 uint8_t satellites_visible = gps->satellites.value();
 uint8_t dgps_numch = satellites_visible;
 uint32_t dgps_age = (uint32_t)gps->location.age();
 uint32_t h_acc=100; //Fixed to 100mm
 uint32_t v_acc=100;
 uint32_t vel_acc = 0;
 uint32_t hdg_acc = 1e5 ; //1 Degree
 uint16_t yaw = (uint16_t)(heading*100.0);


 mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg,
                              time_usec, fix_type,lat,lon, alt, eph, epv,vel, cog,satellites_visible, alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc, yaw);


 //mavlink_msg_gps2_raw_pack(system_id, component_id, &msg,
  //                             time_usec, fix_type, lat, lon,  alt, eph, epv, vel, cog, satellites_visible, dgps_numch, dgps_age, yaw);
 uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

 serWrite(fd,(char*)buf,len);
 mavlink_gps_raw_int_t gps_raw;
 mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
 //printf("Lat:%d, Lon:%d, Yaw:%d\r\n", gps_raw.lat,gps_raw.lon,gps_raw.yaw );
return len;
}
