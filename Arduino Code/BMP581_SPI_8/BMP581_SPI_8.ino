//Gather temp and pressure readings of eight BMP581 sensors over SPI
//Peri Hassanzadeh
//Last Updated: March 8th, 2023

#include <SPI.h>
#include "SparkFun_BMP581_Arduino_Library.h"

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
uint8_t chipSelectPin_1 = 10;
uint8_t chipSelectPin_2 = 3;
uint8_t chipSelectPin_3 = 4;
uint8_t chipSelectPin_4 = 5;
uint8_t chipSelectPin_5 = 6;
uint8_t chipSelectPin_6 = 7;
uint8_t chipSelectPin_7 = 8;
uint8_t chipSelectPin_8 = 9;

uint32_t clockFrequency = 100000;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example2 begin!");

    // Initialize the SPI library
    SPI.begin();

    // Check if sensor is connected and initialize
    // Clock frequency is optional (defaults to 100kHz)
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

void loop()
{
    // Get measurements from the sensor
    bmp5_sensor_data data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8 = {0,0};
    
    int8_t err_1 = pressureSensor_1.getSensorData(&data_1);
    int8_t err_2 = pressureSensor_2.getSensorData(&data_2);
    int8_t err_3 = pressureSensor_3.getSensorData(&data_3);
    int8_t err_4 = pressureSensor_4.getSensorData(&data_4);
    int8_t err_5 = pressureSensor_5.getSensorData(&data_5);
    int8_t err_6 = pressureSensor_6.getSensorData(&data_6);
    int8_t err_7 = pressureSensor_7.getSensorData(&data_7);
    int8_t err_8 = pressureSensor_8.getSensorData(&data_8);

    // Check whether data was acquired successfully
    //Sensor 1
    if(err_1 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_1 (C): ");
        Serial.print(data_1.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_1 (Pa): ");
        Serial.println(data_1.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_1! Error code: ");
        Serial.println(err_1);
    }

    //Sensor 2
    if(err_2 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_2 (C): ");
        Serial.print(data_2.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_2 (Pa): ");
        Serial.println(data_2.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_2! Error code: ");
        Serial.println(err_2);
    }

    //Sensor 3
    if(err_3 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_3 (C): ");
        Serial.print(data_3.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_3 (Pa): ");
        Serial.println(data_3.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_3! Error code: ");
        Serial.println(err_3);
    }

    //Sensor 4
    if(err_4 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_4 (C): ");
        Serial.print(data_4.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_4 (Pa): ");
        Serial.println(data_4.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_4! Error code: ");
        Serial.println(err_4);
    }

    //Sensor 5
    if(err_5 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_5 (C): ");
        Serial.print(data_5.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_5 (Pa): ");
        Serial.println(data_5.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_5! Error code: ");
        Serial.println(err_5);
    }

    //Sensor 6
    if(err_6 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_6 (C): ");
        Serial.print(data_6.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_6 (Pa): ");
        Serial.println(data_6.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_6! Error code: ");
        Serial.println(err_6);
    }

    //Sensor 7
    if(err_7 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_7 (C): ");
        Serial.print(data_7.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_7 (Pa): ");
        Serial.println(data_7.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_7! Error code: ");
        Serial.println(err_7);
    }

    //Sensor 8
    if(err_8 == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature_8 (C): ");
        Serial.print(data_8.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure_8 (Pa): ");
        Serial.println(data_8.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor_8! Error code: ");
        Serial.println(err_8);
    }
    
    // Only print every second
    delay(1000);
}
