/* SAPS CAN Interface Board Firmware
 *  Version 0.1
 * Made by Patrick Murphy
 *
 * Repository https://github.com/Panther-Racing/Vehicle-Surface-Pressure-Sensor-V1
 *
 * Goals: 
 *  use SPI bus to communicate to Surface Pressure Sensor Board, collect data then send data to CAN controller and push to CAN bus.
 *  
 * To Do: 
 *  
 *
 * Program Notes:
 *  9 CS Pins
 *  8 SPI sensors, 1 SPI to CAN transciever
 *  CAN bus rate is variable to Setup
 *  Sensor Wiring is variable to setup
 *
 *
 *  BMP581 notes
 *    data is stored in a 24bit configuration for pressure in the BMP581
        to convert to usable values do [MSB,LSB,XLSB]/2^6 aka >>6
        so to reverse we multiply by 10^6 aka <<6
        uint32_t (pa*2^6)
 *  
 * 64 bits
 * counter(2^4 - 4 bit )
 * 60 bits
 * compound ID (2^4 - 4 bit) 
 * 57 bits
 * CAN MSG 1
 *  PRS 1 2 3
 * 
 * CAN MSG 2
 *  PRS 4 5 6

 * CAN MSG 3
 *  PRS 7 8 tmp 2
 * 
 * 
 * 
 *  workflow for sending data to CAN
 *  determine if sens 1 connected, send data if so with compound ID 1, dont if otherwise
 *  determine if sens 2 connected, send data if so with compound ID 2, dont if otherwise
 *  ...
 *  determine if sens 8 connected, send data if so with compound ID 8, dont if otherwise
 *  restart
 */

#include <SPI.h>
#include <mcp2515.h>
#include "SparkFun_BMP581_Arduino_Library.h"
struct can_frame frame1;
struct can_frame frame2;
struct can_frame frame3;


// Create a new sensor object
  BMP581 pressureSensor_1;
  BMP581 pressureSensor_2;
  BMP581 pressureSensor_3;
  BMP581 pressureSensor_4;
  BMP581 pressureSensor_5;
  BMP581 pressureSensor_6;
  BMP581 pressureSensor_7;
  BMP581 pressureSensor_8;


// SPI parameters all share same MISO(12), MOSI(11) and SCK (13) 
// Each sensor different SS 
  uint8_t chipSelectPin_1 = 3;
  uint8_t chipSelectPin_2 = 4;
  uint8_t chipSelectPin_3 = 5;
  uint8_t chipSelectPin_4 = 6;
  uint8_t chipSelectPin_5 = 7;
  uint8_t chipSelectPin_6 = 8;
  uint8_t chipSelectPin_7 = 9;
  uint8_t chipSelectPin_8 = 10;

  uint32_t clockFrequency = 100000;

//Initial Variable Declarations
  // constants won't change:
  // Interval Declaration
    long interval = 100;  // interval at which to send HZ

    unsigned long previousMillis = 0; 

  
  //8 bit values
    uint8_t TEC=0x00; // transmit error counters
    uint8_t firmwareversion= 0x0A; //Firmware version
      // because firmware version is represented as Vxx.xx, the firmware version needs to be 100 times the value
      // due to unsigned int, ie v0.10 ==> firmware version=10 => 0b00001010 ==> 0x0A
  
    uint8_t counter=0x00; // coms loss check


  // board Chip Select Pin for SPI to CAN chip  
    uint8_t CS=15; // ADC7
    MCP2515 mcp2515(CS);
 


void setup() 
{
  // level shifter enable pins
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);

  interval = 1000/interval; //conversion from HZ to MS

  frame1.can_id  = 0x761; //CAN ID
  frame2.can_id  = 0x761; //CAN ID
  frame3.can_id  = 0x761; //CAN ID

  /*
    Front Wing bot = 
    Frpmt wing top = 
    Rear Wing = 
  */
  
  frame1.can_dlc = 8;//Data lenght of frame
  frame2.can_dlc = 8;//Data lenght of frame
  frame3.can_dlc = 8;//Data lenght of frame
  //debug control    
  //while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();


  while(pressureSensor_1.beginSPI(chipSelectPin_1, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_1 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    while(pressureSensor_2.beginSPI(chipSelectPin_2, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_2 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_3.beginSPI(chipSelectPin_3, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_3 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_4.beginSPI(chipSelectPin_4, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_4 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_5.beginSPI(chipSelectPin_5, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_5 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_6.beginSPI(chipSelectPin_6, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_6 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_7.beginSPI(chipSelectPin_7, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_7 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }
    while(pressureSensor_8.beginSPI(chipSelectPin_8, clockFrequency) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581_8 not connected, check wiring and CS pin!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

  /*  
    pressureSensor_1.beginSPI(chipSelectPin_1, clockFrequency);
    pressureSensor_2.beginSPI(chipSelectPin_2, clockFrequency);
    pressureSensor_3.beginSPI(chipSelectPin_3, clockFrequency);
    pressureSensor_4.beginSPI(chipSelectPin_4, clockFrequency);
    pressureSensor_5.beginSPI(chipSelectPin_5, clockFrequency);
    pressureSensor_6.beginSPI(chipSelectPin_6, clockFrequency);
    pressureSensor_7.beginSPI(chipSelectPin_7, clockFrequency);
    pressureSensor_8.beginSPI(chipSelectPin_8, clockFrequency);
    */
    Serial.println("All BMP581 sensors connected!");
}
  


void messages() {
    
  int8_t state=0x00;
  byte TEC=ReadReg(0x1C); // Reading the Transmit Error Counter(TEC) from the can transciever
  bmp5_sensor_data data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8 = {0,0};
  
  int8_t err_1 = pressureSensor_1.getSensorData(&data_1);
  int8_t err_2 = pressureSensor_2.getSensorData(&data_2);
  int8_t err_3 = pressureSensor_3.getSensorData(&data_3);
  int8_t err_4 = pressureSensor_4.getSensorData(&data_4);
  int8_t err_5 = pressureSensor_5.getSensorData(&data_5);
  int8_t err_6 = pressureSensor_6.getSensorData(&data_6);
  int8_t err_7 = pressureSensor_7.getSensorData(&data_7);
  int8_t err_8 = pressureSensor_8.getSensorData(&data_8);
  //state comparison value

  uint32_t sens1=0x00000000; 
  uint32_t sens2=0x00000000; 
  uint32_t sens3=0x00000000; 
  uint32_t sens4=0x00000000; 
  uint32_t sens5=0x00000000; 
  uint32_t sens6=0x00000000; 
  uint32_t sens7=0x00000000; 
  uint32_t sens8=0x00000000; 
  uint32_t temp2=0x00000000;

  // if sensor ok state=1, if problem state=0
  if(err_1 == BMP5_OK)
  {
    //Serial.println(data_1.pressure);
    
    sens1 = (long(data_1.pressure * 64) & 0x00FFFFE0);
    
    //Serial.println(sens1);

    sens1=sens1>>5;
    //Serial.println(sens1);
  }
  else
  {
    sens1=0x00000000; 
  }

  if(err_2 == BMP5_OK)  
  {
    Serial.println(data_2.pressure);

    sens2 = (long(data_2.pressure *64) & 0x00FFFFE0);
    Serial.println(sens2);
    
    sens2=sens2>>5;
    Serial.println(sens2);

    temp2 = (long(data_2.temperature*65536) & 0x00FFFFE0);
    temp2=temp2>>5;
  }
  else
  {
    temp2=0x00000000;
    sens2=0x00000000;
  }
  
  if(err_3 == BMP5_OK)  
  {
    sens3 = (long(data_3.pressure *64) & 0x00FFFFE0);
    sens3 = sens3>>5;
  }
  else
  {
    sens3=0x00000000;
  }

  if(err_4 == BMP5_OK)  
  {
    sens4 = (long(data_4.pressure *64) & 0x00FFFFE0);
    sens4 = sens4>>5;
    
  }
  else
  {
    sens4 = 0x00000000;
  }
  
  if(err_5 == BMP5_OK)  
  {
    sens5 = (long(data_5.pressure *64) & 0x00FFFFE0);
    sens5=sens5>>5;
  }
  else
  {
    state = state & 0b11011111;
    sens5 = 0x00000000;
  }

  if(err_6 == BMP5_OK)  
  {
    sens6 = (long(data_6.pressure *64) & 0x00FFFFE0);
    sens6=sens6>>5;
  }
  else
  {
    sens6 = 0x00000000;
  }
  
  if(err_7 == BMP5_OK)  
  {
    sens7 = (long(data_7.pressure *64) & 0x00FFFFE0);
    sens7=sens7>>5;
  }
  else
  {
    sens7 = 0x00000000;
  }
  
  if(err_8 == BMP5_OK)  
  {
    sens8 = (long(data_8.pressure *64) & 0x00FFFFE0);
    sens8=sens8>>5;
  }
  else
  {
    sens8 = 0x00000000;
  }
  // compound ID is designed to be a 3 bit value ie 000, 001, 010, 011, 100, 101, 110, or 111
  uint8_t compoundID1= 0x00;
  //compound ID 1
  frame1.data[0] = (((counter<<4)& 0xF0) & (compoundID1<<1 & 0b00001110) & (sens1>>23 & 0x01));
  frame1.data[1] = sens1>>15 & 0xFF;
  frame1.data[2] = sens1>>8 &0xFF;
  frame1.data[3] = (sens1<<1 & 0b11000000) & (sens2>>18 & 0b00111111);
  frame1.data[4] = sens2>>10;
  frame1.data[5] = (sens2>>2 & 0b11000000) & (sens3>>21 & 0b00111111);
  frame1.data[6] = (sens3>> 13 & 0xFF); 
  frame1.data[7] = (sens3>>5 & 0xFF);
  mcp2515.sendMessage(&frame1);

  //compound ID 2
  uint8_t compoundID2= 0x01;
  frame2.data[0] = (((counter<<4)& 0xF0) | (compoundID2<<1 & 0b00001110) | (sens4>>23 & 0x01));
  frame2.data[1] = sens4>>15 & 0xFF;
  frame2.data[2] = sens4>>8 &0xFF;
  frame2.data[3] = (sens4<<1 & 0b11000000) | (sens5>>18 & 0b00111111);
  frame2.data[4] = sens5>>10;
  frame2.data[5] = (sens5>>2 & 0b11000000) | (sens6>>21 & 0b00111111);
  frame2.data[6] = (sens6>> 13 & 0xFF); 
  frame2.data[7] = (sens6>>5 & 0xFF);
  mcp2515.sendMessage(&frame2);

    //compound ID 2
  uint8_t compoundID3= 0x02;
  frame3.data[0] = (((counter<<4)& 0xF0) | (compoundID3<<1 & 0b00001110) | (sens7>>23 & 0x01));
  frame3.data[1] = sens7>>15 & 0xFF;
  frame3.data[2] = sens7>>8 &0xFF;
  frame3.data[3] = (sens7<<1 & 0b11000000) | (sens8>>18 & 0b00111111);
  frame3.data[4] = sens8>>10;
  frame3.data[5] = (sens8>>2 & 0b11000000) | (temp2>>21 & 0b00111111);
  frame3.data[6] = (sens2>> 13 & 0xFF); 
  frame3.data[7] = (sens2>>5 & 0xFF);
  mcp2515.sendMessage(&frame3);
}
void loop()
{

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    // save the last time you sent
    previousMillis = currentMillis;

    //counter setup
    if (counter<15){
      counter=counter+1;// increment counter everytime we send message so we can observe data loss occurence, rolls over at 2^8-1 to 0
    }
    else
    {
      counter=0;
    }

    //Message function
    messages();
  }
}

byte ReadReg (byte Reg){
  // function that reads a single byte message from a specified register, 
  digitalWrite(CS,LOW); // send SOC
  SPI.transfer(0b00000011); //say read to device
  SPI.transfer(Reg); //saying target register
  byte output=SPI.transfer(0x00); //reading from target register
  digitalWrite(CS,HIGH); // send EOC
  return output;
}
