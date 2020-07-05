#include <pigpio.h>
#include "TinyGPS.h"
#include "./MAVLink/common/mavlink.h"

// For efficiency in compiling, this is the only file that uses the
// Eigen lirbary and the 'vector' type we made from it.
// Install libraries first :
//sudo apt-get install libi2c-dev libeigen3-dev libboost-program-options-dev
#include "vector.h"


#include "./IMU/minimu9.h"
#include "./IMU/exceptions.h"
#include "./IMU/pacer.h"
#include "./IMU/imu.h"
#include "./Sonar/ST_Sonar.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <system_error>
#include <chrono>
#include <map>
// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
int SendGPS(TinyGPSPlus *gps,float yaw );
int SendHeartBeat();
int SendParams();
int SendAttitude( float roll, float pitch, float yaw, vector angular_velocity);
int SendData();
uint16_t SendDistance(ST_Sonar::EchoDataType E,float roll,float pitch, float yaw);
bool InitSonar();
void InitParameters();
void SetParameter(mavlink_message_t msg);
