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

int freq = 8000, Lchannel = 0, Rchannel = 1, resolution = 8;

int L = 0;
int R = 0;

Drive drive;

void setup()
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 2, 0);
  Serial.println("Pins configured.");
  ledcSetup(Lchannel, freq, resolution);
  ledcSetup(Rchannel, freq, resolution);
  ledcAttachPin(Lpwm, Lchannel);
  ledcAttachPin(Rpwm, Rchannel);
  pinMode(Ldir, OUTPUT);
  pinMode(Rdir, OUTPUT);
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
    }
    Serial.print("Received: ");
    Serial.println(rxBuffer);

    char *L_index = strchr(rxBuffer, 'L'); // Left
    char *R_index = strchr(rxBuffer, 'R'); // Right
    L = atoi(L_index + 1);
    R = atoi(R_index + 1);
    ledcWrite(Lchannel, abs(L)*2.55);
    Serial.print("L: ");
    Serial.println(L);
    ledcWrite(Rchannel, abs(R)*2.55);
    Serial.print("R: ");
    Serial.println(R);
    if(L > 0)
    {
      digitalWrite(Ldir, HIGH);
    }
    else if(L < 0)
    {
      digitalWrite(Ldir, LOW);
    }
    else
    {
      digitalWrite(Ldir, LOW);
      ledcWrite(Lchannel, 0);
    }
    if(R > 0)
    {
      digitalWrite(Rdir, HIGH);
    }
    else if(R < 0)
    {
      digitalWrite(Rdir, LOW);
    }
    else
    {
      digitalWrite(Rdir, LOW);
      ledcWrite(Rchannel, 0);
    }
    bufferIndex = 0;
  }
}
