#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <string.h>


#include "BoatController.h"
#define SIMULATE_SONAR 1

using namespace std;
char serial_port[]="/dev/ttyS0";
char serial_port_sonar[]="/dev/ttyUSB0";

// ST Sonar settings
ST_Sonar ST;
ST_Sonar::EchoDataType E;
bool SonarAvailable = false;
 

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

 // MAVLink initialisation
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t system_id = 1; // id of computer which is sending the command (ground control software has id of 255, system = 1)
  uint8_t component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t target_component = 0; // Target component, 0 = all (seems to work with 0 or 1)
  mavlink_status_t status;
  int chan = MAVLINK_COMM_0;

  int fd;
  
  TinyGPSPlus *gps = new TinyGPSPlus();
  char serial_buff[1024];
  char serial_msg[1024];
  int chars_read =0;
  
  std::map<std::string, int> parameters;
 
  
int main(int argc, char *argv[])
{


   if (gpioInitialise() < 0) return 1;
   InitParameters();

   SonarAvailable = InitSonar(); 
   
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

  // Set up a timer that expires every 10 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(10000000);
  auto start = std::chrono::steady_clock::now();
  printf("Controller started. Waiting for GPS fix...\r\n");
  auto heartbeat_timer = start;
  auto attitude_timer = start;
  auto sonar_timer = start;
  int MAVByteCount=0;
  float north_total =0;float east_total =0; float roll_total = 0; float pitch_total=0;
  int attitude_samples =0; 
  vector angular_velocity_total;
   angular_velocity_total << 0,0,0;
   
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

    vector angular_velocity = imu.read_gyro(); // Degrees/s
    
    vector acceleration = imu.read_acc(); 
    vector magnetic_field = imu.read_mag(); 
    imu.fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);
    float nx= acceleration(0);float ny = acceleration(1);float nz = acceleration(2);
    float roll =atan2(ny, nz)*60;
    float pitch =atan2(nx,nz)*60;
    matrix m = rotation.toRotationMatrix();
    float northx = m(0,0);
    float eastx =  m(1,0);
    float yaw = atan2(eastx,northx);
    float heading_deg = yaw*(180 / M_PI);
    
    angular_velocity_total += angular_velocity;
    north_total +=northx;east_total += eastx;
    roll_total += roll; pitch_total += pitch;
    attitude_samples++;
     
    chars_read=gpioSerialRead(18,serial_buff,1024);
    if(chars_read>0){
      for(int i=0;i<chars_read;i++)
       {
       gps->encode(serial_buff[i]);
       }
      }
    duration =  start-attitude_timer;    
    // 5e8 = 500 mS
    if(duration.count()>5e8){
       roll = roll_total / attitude_samples;
       pitch = pitch_total / attitude_samples;
       northx = north_total /  attitude_samples;
       eastx = east_total /  attitude_samples;
       yaw = atan2(eastx,northx);
       angular_velocity = angular_velocity_total / attitude_samples;
       SendAttitude( roll, pitch, yaw, angular_velocity);        
       SendGPS(gps,heading_deg);
         
     //  printf("LAT=%f, LON=%f, Age:%05lld, Roll:%.2f,Pitch:%.2f,Yaw:%.2f,Heading:%.2f,Samples:%d\r\n",gps->location.lat(),gps->location.lng(),gps->location.age(),roll,pitch,yaw,heading_deg,attitude_samples);
       north_total =0; east_total =0;  roll_total = 0;  pitch_total=0; attitude_samples =0; angular_velocity_total << 0,0,0;
       attitude_timer = start;
       }
       
       duration =  start-sonar_timer; 
       //2e8 = 200 ms, sonar 5 pings / s   
       if(duration.count()>2e8){
#ifdef SIMULATE_SONAR
 SonarAvailable = true;
#endif
       if (SonarAvailable)
        {
          ST.Scan(&E); 
          SendDistance(E, roll, pitch, yaw );
        }
        sonar_timer = start;
       }
        
      // Check for MAVLink data
      uint8_t stream_id=0;
      uint16_t value =0;
      char id[16];
      while(serDataAvailable(fd) > 0)
      {
      uint8_t byte;
      byte = serReadByte(fd);

      if (mavlink_parse_char(chan, byte, &msg, &status))
         {
         printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);

         switch (msg.msgid) {
	        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  
             printf("- REQUEST_LIST- \r\n");
             SendParams();
           break;
           case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            stream_id = mavlink_msg_request_data_stream_get_req_stream_id(&msg);
            printf("- REQUEST_DATASTREAM: %d \r\n", stream_id);
           break;
           
           case MAVLINK_MSG_ID_PARAM_SET:
            SetParameter( msg);
           break;
           
           } 
         }
      }
  
    try
     {  
     loop_pacer.pace();
     }
     catch(std::exception &e){printf("Error in ' pace' \r\n");}
   }
   gpioTerminate();
}

bool InitSonar()
{
  ST.Params.InitialGain = 50;//0-255
  ST.Params.GainIncrement = 60; //0-255
  ST.Params.LockOut= 60;// Lockout time in 1.96uS [0-65535]
  ST.Params.ScaleDenom=11184 ;
  ST.Params.RangeUnits=1;       // 0=cm, 1= mm units
  ST.Params.MaxDistance = 5000; //Max distance in range units
  ST.Params.TimeOut=ST.Params.MaxDistance;

if((ST.ScannerPort.openDevice(serial_port_sonar,9600)) !=1) 
    {
     printf("Error opening Serial port for Sonar");
     return false;
    }
    else
    { 
     
       if (ST.EstablishCentre())
         {
         ST.EstablishCentre();
         ST.SetStepSize(ST.STEP_HALF);
         ST.UpdateParams();
         ST.Start(60,300);
         }else{return false;}
    }
    return true;
 }

uint16_t SendDistance(ST_Sonar::EchoDataType E, float f_roll, float f_pitch, float f_yaw)
{
   uint32_t time_boot_ms = E.Time;
   uint16_t range=E.Range;
   uint16_t angle = (uint16_t)(E.Angle*10);
   uint16_t roll = (uint16_t) (f_roll*10);
   uint16_t pitch = (uint16_t)(f_pitch*10);
   uint16_t yaw = (uint16_t) (f_yaw*10);
   
   mavlink_msg_scanning_sonar_pack(system_id, component_id,&msg,
                               time_boot_ms, range, angle, roll,pitch,yaw);
   printf("Sonar:T:%ld R:%d, A: %d\r\n",time_boot_ms,range,angle);
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
   serWrite(fd,(char*)buf,len);
   return len;
}

int SendParams()
{
   char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN];
   float param_value = 0;
    uint8_t param_type = MAV_PARAM_TYPE_REAL32;
    uint16_t param_count =parameters.size();
    uint16_t param_index =0;
    uint16_t len=0;
   for(auto& x : parameters)
   {
    std::cout << x.first << "," << x.second << std::endl;
    std::string S = x.first;
    strcpy(param_id,S.c_str()); 

    param_value = (float)x.second;
    
   mavlink_msg_param_value_pack(system_id,component_id, &msg,
                               param_id, param_value, param_type, param_count, param_index);
   param_index++;
   len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

   serWrite(fd,(char*)buf,len);
   }
   return len;
                               
}

int SendData()
{
   
   uint8_t data[MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN];
   static uint16_t seqnr;
   for(int i=0;i<MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN;i++){data[i]=i;}
   seqnr++;
   mavlink_msg_encapsulated_data_pack(system_id, component_id, &msg,
                                seqnr,data);
   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

   serWrite(fd,(char*)buf,len);
   return len;
                           
}

int SendAttitude( float roll_deg,float pitch_deg,float yaw,vector angular_velocity)
{
   //printf("ATT\r\n");
   uint32_t time_boot_ms=timeSinceEpochMillisec();
   float roll = degreesToRadians(roll_deg);
   float pitch = degreesToRadians(pitch_deg);
   float rollspeed = degreesToRadians(angular_velocity(0));
   float pitchspeed= degreesToRadians(angular_velocity(1));
   float yawspeed= degreesToRadians(angular_velocity(2));
   
    mavlink_msg_attitude_pack(system_id, component_id, &msg,
                               time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
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
void SetParameter(mavlink_message_t m_msg)
{
    char id[16];
    uint16_t value = mavlink_msg_param_set_get_param_value(&m_msg);
    uint8_t param_type = mavlink_msg_param_set_get_param_type(&m_msg);
    mavlink_msg_param_set_get_param_id(&m_msg,id);
    parameters[id] = value;
    printf("SET PARAMETER %s to %d \r\n",id,value);
    mavlink_msg_param_value_pack(system_id, component_id, &msg,
                               id, value, param_type, 1, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

    serWrite(fd,(char*)buf,len);
                               
}

void InitParameters()
{
  
  
   parameters["InitialGain"] = 50;
   parameters["GainIncrement"] = 60;
   parameters["LockOut"] = 60;
   parameters["ScaleDenom"] = 11184;
   parameters["RangeUnits"] = 1;  
   parameters["MaxDistance"] = 5000;

}
