#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define Addr_Pressure 0x76 //0x77 if jumper is connected, 0x76 if not
#define Addr_Attitude 0x28 //0x29 if jumper(J1) is connected, 0x28 if not
#define Addr_EEPROM 0x50 //0x50 for 000, 0x57 for 111

#define LED_R 18 //active low
#define LED_G 19 //active low
#define LED_B 20 //active low

#define LoRa_RxPin 1
#define LoRa_TxPin 0
#define GPS_RxPin 5
#define GPS_TxPin 4
#define Sensor_SDA 2
#define Sensor_SCL 3
#define SerialLoRa Serial1
#define SerialGPS Serial2
#define WireSensor Wire1
#define WireQT Wire

#define LoRa_AUX 29 //input
#define LoRa_M0 28 //output, weak pull-up
#define LoRa_M1 27 //output, weak pull-up
/*
  M0  M1  Mode
  0   0   Normal
  0   1   Wake On Radio (transmit)
  1   0   Wake On Radio (receive)
  1   1   Configuration/Deep Sleep
*/
#define Batt_Voltage 26
#define Button 23 //active low

int i2c_read(int device_addr, unsigned int reg_addr){
  Wire.beginTransmission(device_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(device_addr, 1);
  return Wire.read();
}

void i2c_write(int device_addr, unsigned int reg_addr, unsigned int data){
  Wire.beginTransmission(device_addr);
  Wire.write(reg_addr);
  Wire.write(data);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello World");
  SerialLoRa.setRX(LoRa_RxPin);
  SerialLoRa.setTX(LoRa_TxPin);
  SerialLoRa.begin(9600);
  SerialGPS.setRX(GPS_RxPin);
  SerialGPS.setTX(GPS_TxPin);
  SerialGPS.begin(9600);
  WireSensor.setSDA(Sensor_SDA);
  WireSensor.setSCL(Sensor_SCL);
  WireSensor.begin();
  WireQT.setSDA(12);
  WireQT.setSCL(13);
  WireQT.begin();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(LED_R, 255);
  analogWrite(LED_G, 255);
  analogWrite(LED_B, 255);
  // digitalWrite(LED_R, HIGH);
  // digitalWrite(LED_G, HIGH);
  // digitalWrite(LED_B, HIGH);
  delay(1000);
  analogWrite(LED_R, 230);
  analogWrite(LED_G, 230);
  analogWrite(LED_B, 230);
  // digitalWrite(LED_R, LOW);
  // digitalWrite(LED_G, LOW);
  // digitalWrite(LED_B, LOW);
  delay(1000);
  Serial.println("loop");
}
