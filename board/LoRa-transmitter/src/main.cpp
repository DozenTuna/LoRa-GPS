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

#define LoRa_AUX 29
#define LoRa_M0 28
#define LoRa_M1 27
#define Batt_Voltage 26
#define Button 23 //active low

void writeEEPROM(int addr, int data) {
  WireSensor.beginTransmission(Addr_EEPROM);
  WireSensor.write((int)(addr >> 8)); // MSB
  WireSensor.write((int)(addr & 0xFF)); // LSB
  WireSensor.write(data);
  WireSensor.endTransmission();
}
int readEEPROM(int addr) {
  WireSensor.beginTransmission(Addr_EEPROM);
  WireSensor.write((int)(addr >> 8)); // MSB
  WireSensor.write((int)(addr & 0xFF)); // LSB
  WireSensor.endTransmission();
  WireSensor.requestFrom(Addr_EEPROM, 1);
  int data = 0;
  while (WireSensor.available()) {
    data = WireSensor.read();
  }
  return data;
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
  for (size_t i = 0; i < 100; i++)
  {
    writeEEPROM(i,i);
    delay(5);
  }
  Serial.println("EEPROM write complete");
  for (size_t i = 0; i < 100; i++)
  {
    Serial.println(readEEPROM(i));
    delay(5);
  }
  Serial.println("EEPROM read complete");
  delay(1000);
}
