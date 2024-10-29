  //#include <analogWrite.h>
#include <HardwareSerial.h>
#include "arm_IK.h"
#include "drive.h"
//#include "arm_FK.h"

//---ESP32_PIN_CONFIG---
//Arm
#define dir_swivel 4
#define pwm_swivel 18
#define dir_link1 12
#define pwm_link1 27
#define dir_link2 16
#define pwm_link2 17

//Drive
#define Rdir 23
#define Ldir 32
#define Rpwm 19
#define Lpwm 33

//Limit Switch
//#define L1 34
//#define L2 35
//#define L3 15
//#define L4 14
//#define Stow 5
// 1,3,5,6to11,14,15 n0t use cz high when boot or reset
// SERIAL
HardwareSerial SerialPort(1); // use UART0
HardwareSerial Sender(0);
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;


// Pins
const int pwm_pin[] = {pwm_swivel, pwm_link1, pwm_link2, Rpwm, Lpwm};
const int dir_pin[] = {dir_swivel, dir_link1, dir_link2, Rdir, Ldir};
//const int limit_pin[] = {L1, L2, L3, L4, Stow};

// Globals
long prevT = 0;
int changeMode = 0;
int r = 0;
int cx = 0;
int cy = 0;
int cM = 0;

// Arm Variables
int set_link[3];
int pwr[3];
int dir[3];
float linkLength1 = 39.8, linkLength2 = 37.5;
int SW;
int Lone;
int Ltwo;


// Class instances
IK ik;
//FK fk;
Drive drive;

void arm(int x, int y, int z, int Reset)
{
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  //Get Arm controls
  ik.arm_control(x, y, z, deltaT, Reset);
}

void gripper(int grip)
{
  switch (grip)
  {
    case 0: 
      Sender.write('Q');
         //  Serial.println("0");
      break;

    case 1:
      Sender.write('W');
      //      Serial.println("1");
      break;

    case 2: 
      Sender.write('E');
      //      Serial.println("2");
      break;

    case 3: 
      Sender.write('R');
      //      Serial.println("3");
      break;

    case 4: 
      Sender.write('T');
      //      Serial.println("4");
      break;

    case 5: 
      Sender.write('Y');
      //      Serial.println("5");
      break;

    case 6: 
      Sender.write('U');
      //      Serial.println("6");
      break;

    case 7: 
      Sender.write('I');
      //      Serial.println("7");
      break;

    case 8: 
      Sender.write('V');
      //      Serial.println("7");
      break;
    case 9: 
      Sender.write('B');
      //      Serial.println("7");
      break;
    default:
     Sender.write('Q');
        //   Serial.println("7");
     break;
  }
}

void setup()
{
  Wire.begin();
  //setBNO0553();
  Serial.begin(115200);
  Sender.begin(115200) ;   // Use default serial for debug output
  //GPSPort.begin(115200, SERIAL_8N1, 25, 26);
  SerialPort.begin(115200, SERIAL_8N1, 2, 0);//(baud rate,protocol,Tx,Rx)

  // Limit Switch INPUT Pins
//  for (int i = 0; i < 5; i++)
//  {
 
//    pinMode(limit_pin[i], INPUT);
//  }

  // Set Drive Output Pins
  drive.setDrivePins(dir_pin[3], dir_pin[4], pwm_pin[3], pwm_pin[4]);
  ik.setArmPins(dir_pin[0], pwm_pin[0], dir_pin[1], pwm_pin[1], dir_pin[2], pwm_pin[2]);
  Serial.println("Sare pins set hogaye!");
  delay(100);

  ik.set_link_length(linkLength1, linkLength2);

  ik.set_pid();
  Serial.println("PID set");
  delay(10);

//  ik.set_IMU();
//  Serial.println("IMUs connected!!!");
//
//  ik.set_init_coordinates();

}

void loop()
{
  if (SerialPort.available())
  {
    // read the data into the buffer
    while (SerialPort.available())
    {
      rxBuffer[bufferIndex] = (char)SerialPort.read();
      bufferIndex++;
      // Make sure we don't overflow the buffer
      if (bufferIndex >= BUFFER_SIZE)
        bufferIndex = 0;
    }
    //    Serial.println(rxBuffer);

    // Find the positions of the "M", "X", "Y", "P", "Q", "A", "S", "R", "D" and "E" characters in the buffer
    char *M_index = strchr(rxBuffer, 'M'); // gear
    char *x_index = strchr(rxBuffer, 'X'); // drive x
    char *y_index = strchr(rxBuffer, 'Y'); // drive y
    char *P_index = strchr(rxBuffer, 'P'); // arm X
    char *Q_index = strchr(rxBuffer, 'Q'); // arm Y
    char *A_index = strchr(rxBuffer, 'A'); // gripper
    char *S_index = strchr(rxBuffer, 'S'); // arm Z
    char *R_index = strchr(rxBuffer, 'R'); // Reset
    char *D_index = strchr(rxBuffer, 'D'); // Mode
    char *E_index = strchr(rxBuffer, 'E'); // End

    if (M_index != NULL && x_index != NULL && y_index != NULL && P_index != NULL && Q_index != NULL && A_index != NULL && S_index != NULL && R_index != NULL && D_index != NULL && E_index != NULL)
    {
      // Extract the values from the packet
      char m = *(M_index + 1);
      int M = m - '0';
      int x = atoi(x_index + 1);
      int y = atoi(y_index + 1);
      int X = atoi(P_index + 1);
      if (abs(X) < 3)
      {
        X = 0;
      }
      int Y = atoi(Q_index + 1);
      if (abs(Y) < 3)
      {
        Y = 0;
      }
      int grip = atoi(A_index + 1);
      int Z = atoi(S_index + 1);
      if (abs(Z) < 3)
      {
        Z = 0;
      }
      int Reset = atoi(R_index + 1);
      int Mode = atoi(D_index + 1);
      if (Mode == 0)
      {
        if (changeMode == 1)
        {
          
          
          //Serial.println("DRIVEEEEE");
          changeMode = 0;
        }
        ik.beginFK(X, Y, Z);
        gripper(grip);
        //GPSData(GPSPort, SerialPort);
        //forwardkin(Y, Z, X);
      }
      else
    {
      if (changeMode == 0)
      {
        //iksetup();
        ik.set_IMU();
        //Serial.println("IMUs connected!!!");
        ik.set_init_coordinates();
        delay(20);
        ik.drive_rm();
        //forwardkin(Y, Z, X);
        //Serial.println("IK");
        //gripper(grip);
        changeMode = 1;
      }
        arm(X, Y, Z, Reset);
        gripper(grip);
        //GPSData(GPSPort, SerialPort);
        
      }
      delay(10);

    }
    else
    {
      Serial.println("Invalid Packet received");
    }
  }
  char *cM_index = strchr(rxBuffer, 'M');
  char *cx_index = strchr(rxBuffer, 'X'); // drive x
  char *cy_index = strchr(rxBuffer, 'Y');
  char cm = *(cM_index + 1);
  int cM = cm - '0';
  int cx = atoi(cx_index + 1);
  int cy = atoi(cy_index + 1);

  drive.MotorCode(cx, cy, cM);
  bufferIndex = 0;
}
