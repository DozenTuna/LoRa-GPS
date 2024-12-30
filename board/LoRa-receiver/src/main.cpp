#include <Arduino.h>
#include <Wire.h>

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
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  Serial.println("loop");
}
