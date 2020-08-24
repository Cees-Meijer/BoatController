#include "ST_Sonar.h"

ST_Sonar::ST_Sonar()
{
    PosSensor =0;
    StepSize = STEP_FULL;
}

ST_Sonar::~ST_Sonar()
{
    //dtor
}
void ST_Sonar::SetStepSize(stepSize S)
{
if(S==STEP_HALF ){ StepAngleDegrees = 0.9; }
else{ StepAngleDegrees = 1.8; }
Steps = (int)(360/StepAngleDegrees);
StepSize = S;

}

bool ST_Sonar::Start(int SectorFromDegrees, int SectorToDegrees)
{
 Sector1 = SectorFromDegrees/StepAngleDegrees;Sector2=SectorToDegrees/StepAngleDegrees;
 ScanDirection=')';
 ScannerPort.flushReceiver();
 return true;
}

bool ST_Sonar::SendCommand(char m[255],unsigned int len,char Reply)
{
char ch=' ';

usleep(3000); // 3 milliseconds delay

ScannerPort.writeBytes(m,len);
//usleep(100000); //100 mSec
ScannerPort.readChar(&ch,100);
if (Reply==ch) {return true;}
else {return false;};
}

// Read all sonar parameter into the Params struct
bool ST_Sonar::ReadParameters()
{
unsigned char *pChar;
ScannerPort.flushReceiver();
 ScannerPort.writeChar('Q');

 unsigned int bytes= ScannerPort.readBytes(&Params,sizeof(Params),100,1000);

 printf(" Reading %d bytes\r\n",bytes);

 if (bytes< sizeof(Params)){printf("Timeout while reading Parameters\r\n");return false;};
 //unsigned char * P = (unsigned char*)&Params;
 //for(int i=0;i<bytes;i++){printf("%02X ",(unsigned char)*P++);}
 printf("Sonar Parameters");
 printf("Pulse length :%f uS\r\n",Params.TxPulse*1.96 );
 printf("Timeout      :%d\r\n",Params.TimeOut);
 printf("Lockout      :%d\r\n",Params.LockOut);
 printf("Wait Time    :%d\r\n",Params.WaitTime);
 printf("Initial gain :%c\r\n",Params.InitialGain);
 printf("TVG Slope    :%d\r\n",Params.GainIncrement);
 printf("RangeScaling :%d\r\n",Params.ScaleNum);
 printf("RangeScale2  :%d\r\n",Params.ScaleDenom);
 printf("Max distance :%d\r\n",Params.MaxDistance);
 printf("DAC scale 1  :%d\r\n",Params.ScaleDACNum);
 printf("DAC scale 1  :%d\r\n",Params.ScaleDACDenom);
 printf("Range units  :%d\r\n",Params.RangeUnits);
 printf("Checksum     :%d\r\n",Params.Checksum);
 pChar=(unsigned char*)&Params;
 unsigned char CheckSum=0;
 for(unsigned int Index=0;Index< (sizeof(Params)-1);Index++)
 {
 CheckSum = CheckSum + *pChar;
// printf(AnsiString((int)*pChar)+":",(int)CheckSum));
 pChar++;
 }
 printf("Calc Checksum     :%d\r\n",CheckSum);
// tbGain->Position=(int)Params.InitialGain ;
// tbTVG->Position =Params.GainIncrement;
// tbLockOut->Position=Params.LockOut;
return true;
}

bool ST_Sonar::UpdateParams()
{
char Reply;
char C[10];
C[0]='V';Reply=WriteCommand(C,1);
if(StepSize == STEP_FULL){C[0]='Y';} // Step Size: Full = 1.8, Half = 0.9 degrees
else{C[0]='H';}
Reply=WriteCommand(C,1);
C[0]='=';C[1]=0;C[2]=0; // Set motor delay to 0
Reply=WriteCommand(C,3);
C[0]='X'; Reply=WriteCommand(C,1); // Enable TVG
C[0]='K'; Reply=WriteCommand(C,1); // MOde: return Range bin peak value
C[0]='E';C[1]=0xD2; Reply=WriteCommand(C,2);
C[0]='T';C[1]=0x3;C[2]=0x0;C[3]=0x0;C[4]=0x0;C[5]=0x0;C[6]=0x81;C[7]=0x14;C[8]=0x98; //Set TVG Gains
Reply=WriteCommand(C,9);


unsigned char *pChar=(unsigned char*)&Params;
 unsigned char CheckSum=0;
 for(unsigned int Index=0;Index< (sizeof(Params)-1);Index++)
 {
 CheckSum = CheckSum + *pChar;
 pChar++;
 }
Params.Checksum=CheckSum;
ScannerPort.writeChar('J');
ScannerPort.writeBytes(&Params,sizeof(Params));
ScannerPort.readChar(&Reply,100);
printf("Parameters updated:%c ",Reply);
return true;
}

char ST_Sonar::WriteCommand(char*C,int len)
{
char Reply=0;
ScannerPort.writeBytes(C,len);
ScannerPort.readChar(&Reply,100);
return Reply;
}

bool ST_Sonar::Step(bool cw)
{
char Reply;
if (cw) {ScannerPort.writeChar('+');} //Clockwise single step
else {ScannerPort.writeChar('-');}
ScannerPort.readChar(&Reply,100);

if (PosSensor != 0)
 {
 if (Reply!=PosSensor)
  {
  if (PosSensor=='T' || PosSensor=='t')
   {
   DetectedEdge=false;
   }
  else
   {
   DetectedEdge=true;
   }
  }
 }
PosSensor=Reply;

return true; // Returns true when head direction sensor detected.
}
bool ST_Sonar::GoLeft()
{
usleep(DELAYTIME); // Wait for motor. typ 1 ms
//DecScanPosition();
return Step(false); // Call Step CCW, and return result
}

bool ST_Sonar::GoRight()
{
usleep(DELAYTIME); // Wait for motor. typ 1 ms
//IncScanPosition();
return Step(true); // Call Step CW, and return result
}

bool ST_Sonar::EstablishCentre()
{
bool Right,Left,Rising,Falling;
Right=Rising=true;
Left=Falling=false;

unsigned short HalfSensor,Search;
div_t x;
bool Ok;
PosSensor=0; ScanAhead=0;SenseLeft=0;SenseRight=0,Counter=0;
char C[10];
C[0]='H';
Ok=SendCommand(C,1,'H');       // Switch to half step mode
if(Ok==false){return false;}
Search=410;
printf("Rotate Right..\r\n");
if (FindSensorEdge(Right,Rising,Search))
 {
 printf("Found position detector right.\r\n");
 Counter=1;
 if (FindSensorEdge(Right,Falling,Search))
  {
  printf("Found position detector left.\r\n");
  x=div(Counter,2); HalfSensor=x.quot;
  SenseLeft=ScanAhead-HalfSensor;
  }
 else
  {
  Ok=false;
  }
 }
else
 {
 printf("Rotate Left..\r\n");
 if (FindSensorEdge(Left,Rising,Search))
  {
  printf("Found position detector left.\r\n");
  if (FindSensorEdge(Left,Falling,Search))
   {
   GoLeft();
   FindSensorEdge(Right,Rising,Search);
   Counter=1;
   if (!FindSensorEdge(Right,Falling,Search))
    {
    printf("Found position detector left.\r\n");
    Ok=false;
    }
   }
  else { Ok=false; }
  }
 else {Ok=false;}
 }


if (Ok)
 {
 x=div(Counter,2);HalfSensor=x.quot;
 SenseRight=ScanAhead+HalfSensor;
 SenseLeft=SenseRight-Counter+1;
 SenseMiddle=(SenseLeft+SenseRight)/2;
 L0=SenseLeft-2;L1=SenseLeft+2;
 R0=SenseRight+2;R1=SenseRight-2;
 Position=SenseRight+2;
 while (Position>0) { usleep(100000);Position--;GoLeft();}
 printf("Align OK. Pos=%d\r\n",Position);
 if (R1<L1)
  {
  L1=ScanAhead;
  R1=L1;
  }
 HaveLostSteps=false;
 }
else
 {
 HaveLostSteps=true;
 printf("Error while centering head\r\n");
 }
 return Ok;
}

bool ST_Sonar::FindSensorEdge(bool Direction,bool Edge, unsigned short Count)
{
unsigned short Ctr;
bool Ok;
Ctr=0;DetectedEdge=!Edge;
do
 {
 if(Direction) {Ok=GoRight();} else {Ok=GoLeft();}
 Ctr++; Counter++;
 if (Ctr==Count) { printf("Counter overflow\r\n");}
 }
while (Ok && (Ctr!=Count)&&(DetectedEdge!=Edge));
if (Ok && (DetectedEdge==Edge)){return true;} else {return false;}
}

uint ST_Sonar::Scan(EchoDataType *EchoData)
{
 scanState S = SCANNING;
 EchoRangeType Echo;
time_t T = time(0);   // get time now
 struct timespec ts;
 clock_gettime(CLOCK_MONOTONIC, &ts);
 uint32_t timetag_ms = (uint32_t)((uint64_t)(ts.tv_nsec / 1000000) + ((uint64_t)ts.tv_sec * 1000ull));

#ifndef SIMULATE_SONAR
 ScannerPort.flushReceiver();
 ScannerPort.writeChar(ScanDirection); // Step, and take a range-measurement 
 ScannerPort.readBytes(&Echo,sizeof(Echo),15,100);
#endif
#ifdef SIMULATE_SONAR
 Echo.Range = 1000 + Position;
#endif
 T=time(0);
 EchoData->timetag_ms=timetag_ms;
 EchoData->Angle=(uint16_t)((Position*StepAngleDegrees)*10);
 EchoData->Range=Echo.Range;
 //printf("%ld,%0.1f,%d,%d\r\n" ,T,EchoData->Angle,EchoData->Range);

if (ScanDirection == ')')
 {
 Position++;
 if ( Position >= Steps ) { Position=0;S = SCAN_CENTRE;}
 if ( Position== Sector1) { ScanDirection = '('; S = SCAN_SECTOR1;}
  
 }
else
 {
 Position--;
 if ( Position < 0 ) {Position=Steps-1;S = SCAN_CENTRE;}
 if ( Position== Sector2) {ScanDirection = ')';  S = SCAN_SECTOR2;}

 }

 if (Position==1 && Echo.Reply != 't' && Echo.Reply !='T')
  {
   printf("Error in scanhead position.\r\n Restart to recenter.");
  }

return (uint)S;
}
