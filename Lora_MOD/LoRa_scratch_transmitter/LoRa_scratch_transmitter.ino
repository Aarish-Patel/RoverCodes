#include <SPI.h>                                               //the SX127X device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                                          //include the appropriate library  
#include "Settings.h"                                          //include the setiings file, frequencies, LoRa settings etc   

SX127XLT LT;                                                   //create a library class instance called LT

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

const int buff_size = 128;
uint8_t buff[buff_size];
int buffindex = 0;

void loop()
{
  Serial.flush();
  TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array
  buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on reciver

  LT.printASCIIPacket(buff, TXPacketL);                        //print the buffer (the sent packet) as ASCII
  startmS =  millis();                                         //start transmit timer
  if (LT.transmit(buff, TXPacketL, 5000, TXpower, WAIT_TX))    //will return packet length sent if OK, otherwise 0 if transmit error
  {
    endmS = millis();                                          //packet sent, note end time
    TXPacketCount++;
    packet_is_OK();
  }
  else
  {
    packet_is_Error();                                 //transmit packet returned 0, there was an error
  }

  digitalWrite(LED1, LOW);
  Serial.println();
  delay(packet_delay);                                 //have a delay between packets
  if (Serial.available()) {
        char receivedChar = Serial.read();
        if (receivedChar == 'E') {
            // Include 'E' in the buffer
            buff[buffindex++] = receivedChar;
            buffindex = 0; // Reset buffer index for next packet
        } else if (buffindex < buff_size - 1) { // Check if buffer has space
            buff[buffindex++] = receivedChar;
        }
  }
}


void packet_is_OK()
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F("  BytesSent,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  localCRC = LT.CRCCCITT(buff, TXPacketL, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                              //print CRC of sent packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                       //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                         //print total of packets sent OK
}


void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                         //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                    //print IRQ status
  LT.printIrqStatus();                             //prints the text of which IRQs set
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void setup()
{
  pinMode(LED1, OUTPUT);                               //setup pin as output for indicator LED
  led_Flash(2, 125);                                   //two quick LED flashes to indicate program start

  if (VCCPOWER >= 0)
  {
    pinMode(VCCPOWER, OUTPUT);                  //For controlling power to external devices
    digitalWrite(VCCPOWER, LOW);                //VCCOUT on. lora device on
  }

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("3_LoRa_Transmitter_ESP32 Starting"));

  //SPI.begin();                                //default setup can be used be used
  SPI.begin(SCK, MISO, MOSI);                   //alternative format for SPI3, VSPI; SPI.begin(SCK,MISO,MOSI,SS)

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                                   //two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed LED flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
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
  //***************************************************************************************************

  Serial.println();
  LT.printModemSettings();                                //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                            //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);                          //print contents of device registers, normally 0x00 to 0x4F
  Serial.println();
  Serial.println();

  Serial.print(F("Transmitter ready"));
  Serial.println();
}