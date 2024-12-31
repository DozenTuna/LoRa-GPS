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
// #define SerialLoRa Serial1
SoftwareSerial SerialLoRa(LoRa_TxPin, LoRa_RxPin);
#define SerialGPS Serial2
#define WireSensor Wire1
#define WireQT Wire

#define LoRa_AUX 29
#define LoRa_M0 28
#define LoRa_M1 27
int LoRa_Mode = 3;
#define Batt_Voltage 26
#define Button 23 //active low

void setMode(int mode) {
  if (LoRa_Mode != 3)
  {
    while (digitalRead(LoRa_AUX) == LOW)
    {
      delay(1);
    }
    delay(2);
  }
  switch (mode) {
    case 0: //normal mode
      digitalWrite(LoRa_M0, LOW);
      digitalWrite(LoRa_M1, LOW);
      break;
    case 1: //wake up radio (transmit) mode
      digitalWrite(LoRa_M0, HIGH);
      digitalWrite(LoRa_M1, LOW);
      break;
    case 2: //wake on radio (receive) mode
      digitalWrite(LoRa_M0, LOW);
      digitalWrite(LoRa_M1, HIGH);
      break;
    case 3: //configuration / deep sleep mode
      digitalWrite(LoRa_M0, HIGH);
      digitalWrite(LoRa_M1, HIGH);
      break;
    default:
      break;
  }
  LoRa_Mode = mode;
  delay(1);
  
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello World");
  // pinMode(LoRa_RxPin, INPUT);
  // pinMode(LoRa_TxPin, OUTPUT);
  pinMode(LoRa_M0, OUTPUT);
  pinMode(LoRa_M1, OUTPUT);
  pinMode(LoRa_AUX, INPUT);
  // SerialLoRa.setRX(LoRa_RxPin);
  // SerialLoRa.setTX(LoRa_TxPin);
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
  setMode(0);
  int StartingAddr = 0x00;
  SerialLoRa.write(StartingAddr);
  SerialLoRa.write(StartingAddr);
  SerialLoRa.write(StartingAddr);
  SerialLoRa.write(0x70);
  delay(10000);
  Serial.println("loop");
}
