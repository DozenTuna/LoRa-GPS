#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <string.h>

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

void GPSDecode(String data){
  String protocol;
  protocol = data.substring(3, 6);
  if (protocol == "GGA") {
    // Serial.println(data);
    String GGAdata[17];
    int i = 0;
    while (data.indexOf(",") != -1) {
      GGAdata[i] = data.substring(0, data.indexOf(","));
      data = data.substring(data.indexOf(",") + 1);
      i++;
    }
    String UTCHH, UTCMM, UTCSS, LatitudeDD, LatitudeMM, LongitudeDD, LongitudeMM;
    UTCHH = GGAdata[1].substring(0, 2);
    UTCMM = GGAdata[1].substring(2, 4);
    UTCSS = GGAdata[1].substring(4, 10);
    LatitudeDD = GGAdata[2].substring(0, 2);
    LatitudeMM = GGAdata[2].substring(2, 9);
    LongitudeDD = GGAdata[4].substring(0, 3);
    LongitudeMM = GGAdata[4].substring(3, 10);
    if (GGAdata[10] == "M")
    {
      if (GGAdata[12] == "M")
      {
        if (GGAdata[2].indexOf(".") != -1)
        {
          if (GGAdata[4].indexOf(".") != -1)
          {
            Serial.println("GGA Protocol");
            Serial.printf("UTC: %s:%s:%s\n", UTCHH.c_str(), UTCMM.c_str(), UTCSS.c_str());
            Serial.printf("Latitude: %s°%s %s\n", LatitudeDD.c_str(), LatitudeMM.c_str(), GGAdata[3].c_str());
            Serial.printf("Longitude: %s°%s %s\n", LongitudeDD.c_str(), LongitudeMM.c_str(), GGAdata[5].c_str());
            Serial.printf("Fix indicator: %s\n", GGAdata[6].c_str());
            Serial.printf("Number of satellites: %s\n", GGAdata[7].c_str());
            Serial.printf("HDOP: %s\n", GGAdata[8].c_str());
            Serial.printf("Altitude(MSL): %s %s\n", GGAdata[9].c_str(), GGAdata[10].c_str());
            Serial.printf("Geoid separation: %s %s\n", GGAdata[11].c_str(), GGAdata[12].c_str());
            // Serial.printf("Age of differential GPS data: %s\n", GGAdata[13]);
            Serial.println("");
          }
        }
      }
    }

  } else if (protocol == "RMC") {
    String RMCdata[13];
    int i = 0;
    while (data.indexOf(",") != -1) {
      RMCdata[i] = data.substring(0, data.indexOf(","));
      data = data.substring(data.indexOf(",") + 1);
      i++;
    }
    String UTCHH, UTCMM, UTCSS, LatitudeDD, LatitudeMM, LongitudeDD, LongitudeMM, status, Year, Month, Day;
    UTCHH = RMCdata[1].substring(0, 2);
    UTCMM = RMCdata[1].substring(2, 4);
    UTCSS = RMCdata[1].substring(4, 10);
    LatitudeDD = RMCdata[3].substring(0, 2);
    LatitudeMM = RMCdata[3].substring(2, 9);
    LongitudeDD = RMCdata[5].substring(0, 3);
    LongitudeMM = RMCdata[5].substring(3, 11);
    Year = RMCdata[9].substring(4, 6);
    Month = RMCdata[9].substring(2, 4);
    Day = RMCdata[9].substring(0, 2);
    if (Year.toInt() < 70) {
      Year = "20" + Year;
    } else {
      Year = "19" + Year;
    }
    if (RMCdata[2] == "A") {
      status = "Valid";
    } else {
      status = "Invalid";
    }
    Serial.println("RMC Protocol");
    Serial.printf("UTC: %s:%s:%s\n", UTCHH.c_str(), UTCMM.c_str(), UTCSS.c_str());
    Serial.printf("Status: %s\n", status.c_str());
    Serial.printf("Latitude: %s°%s %s\n", LatitudeDD.c_str(), LatitudeMM.c_str(), RMCdata[4].c_str());
    Serial.printf("Longitude: %s°%s %s\n", LongitudeDD.c_str(), LongitudeMM.c_str(), RMCdata[6].c_str());
    Serial.printf("Speed and course over ground: %s %s\n", RMCdata[7].c_str(), RMCdata[8].c_str());
    Serial.printf("Date: %s-%s-%s\n", Year.c_str(), Month.c_str(), Day.c_str());
    Serial.printf("Magnetic variation: %s\n", RMCdata[10].c_str());
    Serial.println("");
  } else if (protocol == "GSA") {
    Serial.print("");
  } else if (protocol == "GSV") {
    Serial.print("");
  } else if (protocol == "VTG") {
    Serial.print("");
  } else if (protocol == "GLL") {
    Serial.print("");
  }
  
}

void setup() {
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
  while (SerialGPS.available()) {
    String data = SerialGPS.readStringUntil(0x0a);
    // Serial.println(data);
    GPSDecode(data);
    delay(10);
  }
  delay(1000);
}
