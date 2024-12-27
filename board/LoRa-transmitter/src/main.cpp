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

signed long int BME280_t_fine;
uint16_t BME280_dig_T1;
int16_t BME280_dig_T2;
int16_t BME280_dig_T3;
uint16_t BME280_dig_P1;
int16_t BME280_dig_P2;
int16_t BME280_dig_P3;
int16_t BME280_dig_P4;
int16_t BME280_dig_P5;
int16_t BME280_dig_P6;
int16_t BME280_dig_P7;
int16_t BME280_dig_P8;
int16_t BME280_dig_P9;
int8_t  BME280_dig_H1;
int16_t BME280_dig_H2;
int8_t  BME280_dig_H3;
int16_t BME280_dig_H4;
int16_t BME280_dig_H5;
int8_t  BME280_dig_H6;
void BME280_readTrim();
void BME280_writeReg(uint8_t BME280_osrs_t, uint8_t BME280_osrs_p, uint8_t BME280_mode, uint8_t BME280_t_sb, uint8_t BME280_filter, uint8_t BME280_spi3w_en, uint8_t BME280_osrs_h);
void BME280_readData(double *temp_act, double *press_act, double *hum_act);
signed long int calibration_T(signed long int adc_T);
unsigned long int calibration_P(signed long int adc_P);
unsigned long int calibration_H(signed long int adc_H);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello World");
  delay(1000);
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
  uint8_t BME280_osrs_t = 1;             //Temperature oversampling x 1
  uint8_t BME280_osrs_p = 1;             //Pressure oversampling x 1
  uint8_t BME280_osrs_h = 1;             //Humidity oversampling x 1
  uint8_t BME280_mode = 3;               //Normal mode
  uint8_t BME280_t_sb = 5;               //Tstandby 1000ms
  uint8_t BME280_filter = 0;             //Filter off 
  uint8_t BME280_spi3w_en = 0;           //3-wire SPI Disable
  
  BME280_writeReg(BME280_osrs_t, BME280_osrs_p, BME280_mode, BME280_t_sb, BME280_filter, BME280_spi3w_en, BME280_osrs_h);
  BME280_readTrim();
}

void loop() {
  double temp_act, press_act, hum_act;
  temp_act = 0.0;
  press_act = 0.0;
  hum_act = 0.0;
  BME280_readData(&temp_act, &press_act, &hum_act);

  Serial.print("TEMP : ");
  Serial.print(temp_act);
  Serial.print(" DegC  PRESS : ");
  Serial.print(press_act);
  Serial.print(" hPa  HUM : ");
  Serial.print(hum_act);
  Serial.println(" %");
  delay(5000);
}

void BME280_readTrim()
{
  uint8_t data[32],i=0;                      // Fix 2014/04/06
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0x88);
  WireSensor.endTransmission();
  WireSensor.requestFrom(Addr_Pressure,24);       // Fix 2014/04/06
  while(WireSensor.available()){
    data[i] = WireSensor.read();
    i++;
  }
  
  WireSensor.beginTransmission(Addr_Pressure);    // Add 2014/04/06
  WireSensor.write(0xA1);                          // Add 2014/04/06
  WireSensor.endTransmission();                    // Add 2014/04/06
  WireSensor.requestFrom(Addr_Pressure,1);        // Add 2014/04/06
  data[i] = WireSensor.read();                     // Add 2014/04/06
  i++;                                       // Add 2014/04/06
  
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0xE1);
  WireSensor.endTransmission();
  WireSensor.requestFrom(Addr_Pressure,7);        // Fix 2014/04/06
  while(WireSensor.available()){
    data[i] = WireSensor.read();
    i++;    
  }
  BME280_dig_T1 = (data[1] << 8) | data[0];
  BME280_dig_T2 = (data[3] << 8) | data[2];
  BME280_dig_T3 = (data[5] << 8) | data[4];
  BME280_dig_P1 = (data[7] << 8) | data[6];
  BME280_dig_P2 = (data[9] << 8) | data[8];
  BME280_dig_P3 = (data[11]<< 8) | data[10];
  BME280_dig_P4 = (data[13]<< 8) | data[12];
  BME280_dig_P5 = (data[15]<< 8) | data[14];
  BME280_dig_P6 = (data[17]<< 8) | data[16];
  BME280_dig_P7 = (data[19]<< 8) | data[18];
  BME280_dig_P8 = (data[21]<< 8) | data[20];
  BME280_dig_P9 = (data[23]<< 8) | data[22];
  BME280_dig_H1 = data[24];
  BME280_dig_H2 = (data[26]<< 8) | data[25];
  BME280_dig_H3 = data[27];
  BME280_dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
  BME280_dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F); // Fix 2014/04/06
  BME280_dig_H6 = data[31];                                   // Fix 2014/04/06
}

void BME280_writeReg(uint8_t BME280_osrs_t, uint8_t BME280_osrs_p, uint8_t BME280_mode, uint8_t BME280_t_sb, uint8_t BME280_filter, uint8_t BME280_spi3w_en, uint8_t BME280_osrs_h)
{
  uint8_t ctrl_meas_reg = (BME280_osrs_t << 5) | (BME280_osrs_p << 2) | BME280_mode;
  uint8_t config_reg    = (BME280_t_sb << 5) | (BME280_filter << 2) | BME280_spi3w_en;
  uint8_t ctrl_hum_reg  = BME280_osrs_h;
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0xF2);
  WireSensor.write(ctrl_hum_reg);
  WireSensor.endTransmission();
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0xF4);
  WireSensor.write(ctrl_meas_reg);
  WireSensor.endTransmission();
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0xF5);
  WireSensor.write(config_reg);
  WireSensor.endTransmission();
}

void BME280_readData(double *temp_act, double *press_act, double *hum_act)
{
  int i = 0;
  uint32_t data[8];
  signed long int temp_raw, pres_raw, hum_raw;
  WireSensor.beginTransmission(Addr_Pressure);
  WireSensor.write(0xF7);
  WireSensor.endTransmission();
  WireSensor.requestFrom(Addr_Pressure,8);
  while(WireSensor.available()){
    data[i] = WireSensor.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
  *temp_act = (double)calibration_T(temp_raw) / 100.0;
  *press_act = (double)calibration_P(pres_raw) / 100.0;
  *hum_act = (double)calibration_H(hum_raw) / 1024.0;
}

signed long int calibration_T(signed long int adc_T)
{
  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)BME280_dig_T1<<1))) * ((signed long int)BME280_dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)BME280_dig_T1)) * ((adc_T>>4) - ((signed long int)BME280_dig_T1))) >> 12) * ((signed long int)BME280_dig_T3)) >> 14;
  
  BME280_t_fine = var1 + var2;
  T = (BME280_t_fine * 5 + 128) >> 8;
  return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)BME280_t_fine)>>1) - (signed long int)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)BME280_dig_P6);
  var2 = var2 + ((var1*((signed long int)BME280_dig_P5))<<1);
  var2 = (var2>>2)+(((signed long int)BME280_dig_P4)<<16);
  var1 = (((BME280_dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)BME280_dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int)BME280_dig_P1))>>15);
  if (var1 == 0)
  {
    return 0;
  }    
  P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
  if(P<0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);   
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;    
  }
  var1 = (((signed long int)BME280_dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
  var2 = (((signed long int)(P>>2)) * ((signed long int)BME280_dig_P8))>>13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + BME280_dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;
  v_x1 = (BME280_t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) -(((signed long int)BME280_dig_H4) << 20) - (((signed long int)BME280_dig_H5) * v_x1)) + 
        ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)BME280_dig_H6)) >> 10) * 
        (((v_x1 * ((signed long int)BME280_dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
        ((signed long int) BME280_dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)BME280_dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);   
}
