#define NSS 5                                   //select pin on LoRa device
#define SCK 18                                  //SCK on SPI3
#define MISO 19                                 //MISO on SPI3 
#define MOSI 23                                 //MOSI on SPI3 

#define NRESET 27                               //reset pin on LoRa device
#define LED1 2                                  //on board LED, high for on
#define DIO0 35                                 //DIO0 pin on LoRa device, used for RX and TX done 
#define VCCPOWER 14                             //pin controls power to external devices

#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using


const uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const int8_t TXpower = 10;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 1000;             //mS delay between packets
