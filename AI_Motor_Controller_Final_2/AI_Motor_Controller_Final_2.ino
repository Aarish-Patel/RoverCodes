#include <HardwareSerial.h>

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

unsigned long lastReceivedTime = 0;
const unsigned long timeoutDuration = 1000;

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
  delay(400);
}

void loop()
{
  if (SerialPort.available())
  {
    lastReceivedTime = millis();
    while (SerialPort.available())
    {
      char receivedChar = (char)SerialPort.read();

      if (bufferIndex < BUFFER_SIZE - 1)
      {
        rxBuffer[bufferIndex++] = receivedChar;
      }
    }
    rxBuffer[bufferIndex] = '\0';
    Serial.print("Received: ");
    Serial.println(rxBuffer);

    char *L_index = strchr(rxBuffer, 'L');
    char *R_index = strchr(rxBuffer, 'R');

    if (L_index != NULL && R_index != NULL)
    {
      L = atoi(L_index + 1);
      R = atoi(R_index + 1);
    }
    else
    {
      stopMotors();
      Serial.println("Invalid data received! Motors stopped.");
    }

    bufferIndex = 0;
  }

  if (millis() - lastReceivedTime > timeoutDuration)
  {
    stopMotors();
    Serial.println("Timeout! Motors stopped.");
  }

  updateMotorControl(L, R);
}

void updateMotorControl(int L, int R)
{
  ledcWrite(Lchannel, abs(L) * 2.55);
  if (L > 0)
    digitalWrite(Ldir, LOW);
  else if (L < 0)
    digitalWrite(Ldir, HIGH);
  else
    digitalWrite(Ldir, LOW);
  ledcWrite(Rchannel, abs(R) * 2.55);
  if (R > 0)
    digitalWrite(Rdir, LOW);
  else if (R < 0)
    digitalWrite(Rdir, HIGH);
  else
    digitalWrite(Rdir, LOW);
}

void stopMotors()
{
  ledcWrite(Lchannel, 0);
  ledcWrite(Rchannel, 0);
  digitalWrite(Ldir, LOW);
  digitalWrite(Rdir, LOW);
  L = 0;
  R = 0;
}
