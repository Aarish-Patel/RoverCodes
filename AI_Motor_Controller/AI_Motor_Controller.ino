#include <HardwareSerial.h>
#include "drive.h"

#define Rdir 23
#define Ldir 32
#define Rpwm 19
#define Lpwm 33

HardwareSerial SerialPort(1);

const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
int bufferIndex = 0;

const int pwm_pin[] = {Rpwm, Lpwm};
const int dir_pin[] = {Rdir, Ldir};

int L = 0;
int R = 0;

Drive drive;

void setup()
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 2, 0);
  drive.setDrivePins(dir_pin[0], dir_pin[1], pwm_pin[0], pwm_pin[1]);
  Serial.println("Pins configured.");
  delay(100);
}

void loop()
{
  if (SerialPort.available())
  {
    while (SerialPort.available())
    {
      char receivedChar = (char)SerialPort.read();

      if (bufferIndex < BUFFER_SIZE - 1)
      {
        rxBuffer[bufferIndex++] = receivedChar;
      }
      drive.MotorCode(L, R); 
    }
    Serial.print("Received: ");
    Serial.println(rxBuffer);

    char *L_index = strchr(rxBuffer, 'L'); // Left
    char *R_index = strchr(rxBuffer, 'R'); // Right

    if (L_index != NULL && R_index != NULL)
    {
      L = atoi(L_index + 1);
      R = atoi(R_index + 1);
      drive.MotorCode(L, R);
    }
    else
    {
      Serial.println("Invalid Packet received.");
    }
    bufferIndex = 0;
  }
  else
  {
    drive.Stop();
  }
}
