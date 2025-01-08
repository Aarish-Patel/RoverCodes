#include <Wire.h>
#include "displayUtils.h"
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
SX127XLT LT;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;
int bufferIndex = 0;
uint8_t RXPacketL;                              //used to store the received packet length
uint8_t RXBUFFER[100];
char raw_packet[40];
uint8_t int_raw_packet[40];
String input = "";
bool sweepInputMode = false;
bool freqInputMode = false;
bool abortInputMode = false;
bool rtbInputMode = false;
bool latitudeInputMode = false;
bool longitudeInputMode = false;
uint8_t uint8_packet[sizeof(raw_packet)];
uint32_t RXpacketCount;
uint32_t errors;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000;
const int BUFFER_SIZE = 128;
char rxBuffer[BUFFER_SIZE];
void gripper(int grip)
{
  switch (grip)
  {
    case 0: 
      Serial.write('Q');
         //  Serial.println("0");
      break;

    case 1:
      Serial.write('W');
      //      Serial.println("1");
      break;

    case 2: 
      Serial.write('E');
      //      Serial.println("2");
      break;

    case 3: 
      Serial.write('R');
      //      Serial.println("3");
      break;

    case 4: 
      Serial.write('T');
      //      Serial.println("4");
      break;

    case 5: 
      Serial.write('Y');
      //      Serial.println("5");
      break;

    case 6: 
      Serial.write('U');
      //      Serial.println("6");
      break;

    case 7: 
      Serial.write('I');
      //      Serial.println("7");
      break;

    case 8: 
      Serial.write('V');
      //      Serial.println("7");
      break;
    case 9: 
      Serial.write('B');
      //      Serial.println("7");
      break;
    default:
     Serial.write('Q');
        //   Serial.println("7");
     break;
  }
}
void getvals() {
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
      Serial.print(M);
      Serial.print('_');
      Serial.print(x);
      Serial.print('_');
      Serial.print(y);
      Serial.print('_');
      Serial.print(X);
      Serial.print('_');
      Serial.print(Y);
      Serial.print('_');
      gripper(grip);
      Serial.print('_');
      Serial.print(Z);
      Serial.print('_');
      Serial.print(Reset);
      Serial.print('_');
      Serial.print(Mode);
      Serial.print('E');
    }
    else
    {
      Serial.println("Invalid Packet received");
    }
}
void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;

  IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register

  RXpacketCount++;

  printElapsedTime();                             //print elapsed time to Serial Monitor
  Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);       //print the packet as ASCII characters

  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
  Serial.print(F(",CRC,"));                       //contents, not the LoRa device internal CRC
  Serial.print(localCRC, HEX);
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
  Serial.println();
}
void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
    Serial.println();
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
    Serial.println();
  }

}
void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}
void convertReceivedPacketToBuffer(uint8_t* RXPacket, uint8_t RXPacketL) {
  // Clear the buffer before storing the new packet
  memset(rxBuffer, 0, BUFFER_SIZE);
  bufferIndex = 0;

  // Copy the received packet into rxBuffer
  for (int i = 0; i < RXPacketL && bufferIndex < BUFFER_SIZE - 1; i++) {
    rxBuffer[bufferIndex++] = static_cast<char>(RXPacket[i]);
  }
  rxBuffer[bufferIndex] = '\0';
}

void convertCharToUint8(const char* charArray, int length, uint8_t* uint8Array) {
  for (int i = 0; i < length; ++i) {
    uint8Array[i] = static_cast<uint8_t>(charArray[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Remote_ESP_init!");
  SPI.begin(SCK, MISO, MOSI, NSS);
  LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE);
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on RX done
}

void recevent() {
  if (RXPacketL = LT.receive(RXBUFFER, sizeof(RXBUFFER), 5000, WAIT_RX)) { //wait for a packet to arrive with 2 second (2000mS) timeout
    PacketRSSI = LT.readPacketRSSI();  //read the received RSSI value
    PacketSNR = LT.readPacketSNR();    //read the received SNR value
    if (RXPacketL == 0) {  //if the LT.receive() function detects an error, RXpacketL is 0
      packet_is_Error();
    } else {
      packet_is_OK();
      convertReceivedPacketToBuffer(RXBUFFER, RXPacketL);
      Serial.println(rxBuffer);
      getvals();
    }
  }
}


void loop() {
}