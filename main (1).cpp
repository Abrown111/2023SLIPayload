#include <Arduino.h>
#include <SD.h>
#include <esp_now.h>
#include <SPI.h>
#include <Wire.h>

/**PIN DEFINITIONS**/
#define LED_PIN 27 //Pin for LED
#define IMU_CS 16 //CS Pin for the MPU-6500 IMU
#define ADC_CS 25 //CS Pin for the MCP356 ADC
#define SD_CS 4 //CS for SD Card adaptor

/*IMPORTANT IMU REGISTERS*/
#define ACCEL_CONFIG 0x1C //Config Register. 28 in decimal
#define ACCEL_XOUT_H 0x3B //Beginning of 3 16bit messages for Accelerometer (MSB). 59 in decimal
#define GYRO_XOUT_H 0x43 //Beginning of 3 16bit messages for Gyroscope (MSB). 67 in decimal

/*IMPORTANT ADC REGISTERS*/
#define ADC_DATA 0x0 //Data out register. Returns 4/24/32 bit data according to CONFIG3 register. Default output is 24 bits
#define MUX_VIN 0x6 //Multiplexer selection. Data input determines which channels data will be recieved from
#define CONFIG3_REG 0x4 //Configuration register to determine output of the ADC. 
#define CONFIG0_REG 0x1 //Configuration register to select external/internal clock source

/*RESOLUTIONS FOR ACCELEROMETER AND GYROSCOPE*/
int ACC_RES = 2048; 
int GYRO_RES = 131;

/*FINAL DATA OUTPUT STRUCTURE*/

double ADC;
String outputData;


/*SD CARD FILE*/
File file;


/*UNION DATA TYPE FOR IMU VALUES*/
union IMUData{ 
  struct
   {
      struct
      {
        uint8_t LOWER; //Lower byte for X values
        uint8_t HIGHER; //Higher byte for X values
      }X;

      struct
      {
        uint8_t LOWER; //Lower byte for Y values
        uint8_t HIGHER; //Higher byte for Y values
      }Y;

      struct
      {
        uint8_t LOWER; //Lower byte for Y values
        uint8_t HIGHER; //Higher byte for Y values
      }Z;

      struct
      {
        double X; //Complemented and scaled X values (g)
        double Y; //Complemented and scaled Y values (g)
        double Z; //Complemented and scaled Z values (g)
      }DATA;

    }ACCEL;

  struct
    {
      struct
      {
        uint8_t LOWER; //Lower byte for X values
        uint8_t HIGHER; //Higher byte for X values
      }X;

      struct
      {
        uint8_t LOWER; //Lower byte for Y values
        uint8_t HIGHER; //Higher byte for Y values
      }Y;

      struct
      {
        uint8_t LOWER; //Lower byte for Z values
        uint8_t HIGHER; //Higher byte for Z values
      }Z;

      struct
      {
        double X; //Complemented and scaled X values (deg/s)
        double Y; //Complemented and scaled Y values (deg/s)
        double Z; //Complemented and scaled Z values (deg/s)
      }DATA;

    }GYRO;
};

IMUData imu;

union uint24_t{
  struct
  {
    uint8_t LOWER;
    uint8_t MIDDLE;
    uint8_t UPPER;
  }BYTE;
};




double twosComp(double val, int bitCount){
  double bytes = val;
  if(val > (2^(bitCount-1))-1){
    bytes -= 2^bitCount;
  }
  return bytes;
}


void writeReg(uint8_t reg, uint8_t data, int csPin){

  if(csPin == IMU_CS){
    digitalWrite(csPin, LOW);
    SPI.transfer(reg);
    SPI.transfer(data);
    digitalWrite(csPin, HIGH);
  }
  
  else if(csPin == ADC_CS){
    uint8_t writeByte = 0b01000010;
    digitalWrite(csPin, LOW);
    SPI.transfer((reg << 2)| writeByte);
    SPI.transfer(data);
    digitalWrite(csPin, HIGH);
  }

  else{
    Serial.println("Error: CS Pin not recognized (W)");
  }
}

signed long readReg(uint8_t reg, int csPin, bool isExtended = false){


  if(csPin == IMU_CS){
    char data;
    digitalWrite(csPin, LOW);
    SPI.transfer(0x80 | reg);
    data = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);

    return data;
  }

  else if(csPin == ADC_CS){
    uint8_t readByte = 0b01000010;
    if(isExtended){
      uint32_t data;
      digitalWrite(csPin, LOW);
      SPI.transfer((reg << 2)| readByte);
      data = SPI.transfer32(0x00);
      digitalWrite(csPin, HIGH);

      return data;
    }else{
      
      uint24_t data;
      digitalWrite(csPin, LOW);
      SPI.transfer((reg << 2)| readByte);
      data.BYTE.LOWER = SPI.transfer(0x00);
      data.BYTE.MIDDLE = SPI.transfer(0x00);
      data.BYTE.UPPER = SPI.transfer(0x00);
      digitalWrite(csPin, HIGH);

      return ((data.BYTE.UPPER << 16) | (data.BYTE.MIDDLE << 8) | (data.BYTE.LOWER));
    }
  }

  else{
    Serial.println("Error: CS Pin not recognized (R)");
    return 0;
  }

}

IMUData getIMUData(){
  IMUData IMUDATA;
  IMUDATA.ACCEL.X.HIGHER = readReg(ACCEL_XOUT_H, IMU_CS);
  IMUDATA.ACCEL.X.LOWER = readReg(ACCEL_XOUT_H + 1, IMU_CS);
  IMUDATA.ACCEL.Y.HIGHER = readReg(ACCEL_XOUT_H + 2, IMU_CS);
  IMUDATA.ACCEL.Y.LOWER = readReg(ACCEL_XOUT_H + 3, IMU_CS);
  IMUDATA.ACCEL.Z.HIGHER = readReg(ACCEL_XOUT_H + 4, IMU_CS);
  IMUDATA.ACCEL.Z.LOWER = readReg(ACCEL_XOUT_H + 5, IMU_CS);

  IMUDATA.ACCEL.DATA.X = twosComp((IMUDATA.ACCEL.X.HIGHER << 8) | IMUDATA.ACCEL.X.LOWER, 16)/ACC_RES;
  IMUDATA.ACCEL.DATA.Y = twosComp((IMUDATA.ACCEL.Y.HIGHER << 8) | IMUDATA.ACCEL.Y.LOWER, 16)/ACC_RES;
  IMUDATA.ACCEL.DATA.Z = twosComp((IMUDATA.ACCEL.Z.HIGHER << 8) | IMUDATA.ACCEL.Z.LOWER, 16)/ACC_RES;

  IMUDATA.GYRO.X.HIGHER = readReg(GYRO_XOUT_H, IMU_CS);
  IMUDATA.GYRO.X.LOWER = readReg(GYRO_XOUT_H + 1, IMU_CS);
  IMUDATA.GYRO.Y.HIGHER = readReg(GYRO_XOUT_H + 2, IMU_CS);
  IMUDATA.GYRO.Y.LOWER = readReg(GYRO_XOUT_H + 3, IMU_CS);
  IMUDATA.GYRO.Z.HIGHER = readReg(GYRO_XOUT_H + 4, IMU_CS);
  IMUDATA.GYRO.Z.LOWER = readReg(GYRO_XOUT_H + 5, IMU_CS);

  IMUDATA.GYRO.DATA.X = twosComp((IMUDATA.GYRO.X.HIGHER << 8) | IMUDATA.GYRO.X.LOWER, 16)/GYRO_RES;
  IMUDATA.GYRO.DATA.Y = twosComp((IMUDATA.GYRO.Y.HIGHER << 8) | IMUDATA.GYRO.Y.LOWER, 16)/GYRO_RES;
  IMUDATA.GYRO.DATA.Z = twosComp((IMUDATA.GYRO.Z.HIGHER << 8) | IMUDATA.GYRO.Z.LOWER, 16)/GYRO_RES;
  return IMUDATA;
}


void setup(){
  Serial.begin(115200);
  SPI.begin();
  pinMode(IMU_CS, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ADC_CS, OUTPUT);

  digitalWrite(LED_PIN, HIGH);

  /*CONFIGURATIONS FOR IMU AND ADC*/
  writeReg(ACCEL_CONFIG, 0b00011000, IMU_CS); //Writes binary 11000 to ACCEL_CONFIG register. According to datasheet, 0b11 on bits [4:3] will allow for 16g accelerometer range
  writeReg(CONFIG0_REG, 0b00100000, ADC_CS); //Writes binary 100000 to CONFIFG0 register. According to the datasheet, this selects the clock source to internal oscillator.
  SD.begin(15);

  SD.remove("/T17SubscaleData.csv");
  file = SD.open("/T17SubscaleDataModule1.csv", FILE_WRITE);
  file = SD.open("/T17SubscaleDataModule1.csv", FILE_APPEND);
  file.println(String("Accelerometer X\t") + String("Accelerometer Y\t") + String("Accelerometer Z\t") + String("Quaternion\t") + String("Altitude (Ft)\t") + 
              String("Power (mW)\t") + String("GPS Coordinates\t"));
  file.close();

}

void loop(){

  // imu = getIMUData();
  // ADC = 3.3*twosComp(readReg(ADC_DATA, ADC_CS), 24)/8388608;
  // outputData = String(imu.ACCEL.DATA.X) + "\t" + String(imu.ACCEL.DATA.Y) + "\t" + String(imu.ACCEL.DATA.Z) + "\t";
  // outputData += String(imu.GYRO.DATA.X) + "\t" + String(imu.GYRO.DATA.Y) + "\t" + String(imu.GYRO.DATA.Z) + "\t ";
  // outputData += "Voltage " + String(ADC);
  char data[2];
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(0x80 | ACCEL_XOUT_H);
  data[0] = SPI.transfer(0x00);
  data[1] = SPI.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);

  //double please = ((readReg(ACCEL_XOUT_H, IMU_CS) << 8)|readReg(ACCEL_XOUT_H + 1, IMU_CS))/16384;
  double please = twosComp((data[0]<<8)|data[1], 16)/16384;
  Serial.println(please);
  // file = SD.open("/T17SubscaleDataModule1.csv", FILE_APPEND);
  // file.println(outputData);
  // file.close();



}