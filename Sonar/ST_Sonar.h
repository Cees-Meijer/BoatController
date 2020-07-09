#ifndef ST_SONAR_H
#define ST_SONAR_H
#include <serialib.h>
#define DELAYTIME 1000

class ST_Sonar
{
    public:
        ST_Sonar();
        virtual ~ST_Sonar();



#pragma pack(1)
typedef struct
{
 unsigned short TxPulse;
 unsigned short TimeOut;
 unsigned short LockOut;
 unsigned short WaitTime;
 unsigned char InitialGain;
 unsigned short GainIncrement;
 unsigned short ScaleNum;
 unsigned short ScaleDenom;
 unsigned short MaxDistance;
 unsigned short ScaleDACNum;
 unsigned short ScaleDACDenom;
 unsigned char RangeUnits;
 unsigned char Checksum;
}ParamsType;
typedef struct{
 unsigned short Range;
 char Reply;
 }EchoRangeType;
#pragma pack()
typedef struct{
 uint32_t timetag_ms; // milliseconds since startup
 uint16_t Angle;      //in 0.1 degrees
 uint16_t Range; // cm or mm, depending on setting sonar
 float X;
 float Y;
 }EchoDataType;
ParamsType Params;

enum stepSize {STEP_FULL,STEP_HALF};
enum scanState {SCANNING,SCAN_SECTOR1,SCAN_SECTOR2,SCAN_CENTRE};

bool SaveSettings();
char WriteCommand(char*C,int len);
bool ReadSettings();
bool ReadParameters();
bool SendCommand(char m[255],unsigned int len,char Reply);
bool UpdateParams();
bool Step(bool cw);
bool GoLeft();
bool GoRight();
bool EstablishCentre();
bool FindSensorEdge(bool Direction,bool Edge, unsigned short Count);
uint Scan(EchoDataType*ED);
bool Start(int,int);
void SetStepSize(stepSize);
bool DetectedEdge,HaveLostSteps;
unsigned short Counter,SenseLeft, SenseRight,SenseMiddle,ScanAhead,L0,L1,R0,R1;

char PosSensor;
int Steps,Position,Sector1,Sector2;
serialib ScannerPort;

    protected:

    private:
float StepAngleDegrees;
stepSize StepSize;
char ScanDirection;
};

#endif // ST_SONAR_H
