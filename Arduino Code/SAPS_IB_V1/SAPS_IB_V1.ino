/* SAPS CAN Interface Board
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
 *    data is stored in a 24bit configuration for both temp and pressure so need 6 data points for each if want temp and pressure
 * Message N
 *  bit 0: compound ID N
 *  bit 1: counter
 *  bit 2: sens N P_XLSB 
 *  bit 3: sens N P_LSB
 *  bit 4: sens N P_MSB
 *  bit 5: sens N T_XLSB
 *  bit 6: sens N T_LSB
 *  bit 7: sens N T_MSB
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
struct can_frame frame;

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
  uint8_t chipSelectPin_1 = 1;
  uint8_t chipSelectPin_2 = 2;
  uint8_t chipSelectPin_3 = 9;
  uint8_t chipSelectPin_4 = 10;
  uint8_t chipSelectPin_5 = 11;
  uint8_t chipSelectPin_6 = 12;
  uint8_t chipSelectPin_7 = 13;
  uint8_t chipSelectPin_8 = 14;

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
    uint8_t CS=22; // ADC7
    MCP2515 mcp2515(CS);
 


void setup() 
{
  pinMode(A6,INPUT_PULLUP);
  pinMode(A7,INPUT_PULLUP);
  interval = 1000/interval; //conversion from HZ to MS
  //Setting Pullups on ADC pins in use


  frame.can_id  = 0x761; //CAN ID
  /*
    Front Wing = 
    Side Wing = 
    Rear Wing = 
  */
  
  frame.can_dlc = 8;//Data lenght of frame
  //debug control    
  while (!Serial);
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
    Serial.println("All BMP581 sensors connected!");
}
  


void message1() {

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
  
  // if sensor ok state=1, if problem state=0
  if(err_1 == BMP5_OK)  
  {
    state = state || 0x01;
  }
  else
  {
    state = state and 0b11111110; 
  }

  if(err_2 == BMP5_OK)  
  {
    state = state | 0b00000010;
  }
  else
  {
    state = state & 0b11111101;
  }
  
  if(err_3 == BMP5_OK)  
  {
    state = state | 0b00000100;
  }
  else
  {
    state = state & 0b11111011;
  }

  if(err_4 == BMP5_OK)  
  {
    state = state | 0b00001000;
  }
  else
  {
    state = state & 0b11110111;
  }
  
  if(err_5 == BMP5_OK)  
  {
    state = state | 0b00010000;
  }
  else
  {
    state = state & 0b11101111;
  }

  if(err_6 == BMP5_OK)  
  {
    state = state | 0b00100000;
  }
  else
  {
    state = state & 0b11011111;
  }
  
  if(err_7 == BMP5_OK)  
  {
    state = state | 0b01000000;
  }
  else
  {
    state = state & 0b10111111;
  }
  
  if(err_8 == BMP5_OK)  
  {
    state = state | 0b10000000;
  }
  else
  {
    state = state & 0b01111111;
  }

  frame.data[0] = state;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = firmwareversion;
  frame.data[6] = TEC;
  frame.data[7] = counter; 

  mcp2515.sendMessage(&frame);
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
    message1();

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
