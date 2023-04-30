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
 *  CAN bus rate is variable to Setup
 *  Sensor Wiring is variable to setup
 *
 */


#include <SPI.h>
#include <mcp2515.h>
struct can_frame frame;

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
 


void setup() {
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
  
}

void message1() {
  //ADC Value Aquisition

  byte TEC=ReadReg(0x1C); // Reading the Transmit Error Counter(TEC) from the can transciever


  frame.data[0] = 
  frame.data[1] = 
  frame.data[2] = 
  frame.data[3] = 
  frame.data[4] = 
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
