#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include <TinyGPS.h>
//#include <SD.h>
#include <ReefwingAHRS.h>
#include <SPI.h>
#include "main.h"
#include "rf.h"
#include "bmp388.h"
#include "BNO055.h"
//#include "gps.h"
//#include "MMC5603.h"

void motionCalRawPrint(BNO_DOF3_Float accData, BNO_DOF3_Float gyroData, BNO_DOF3_Float magData);

// Ana algoritma devresi için genel obje tanımlamaları değişken tanımları
bool RampaDeger = true;
bool IrtifaSinir = false;
bool KurtarmaSistemi = false;
float rampaDegerIrtifa = 0;
float altitude;
float pitch, roll;

float altbuf[40];
float buftoplam = 0;
float bufort = 0;
int buffcnt = 0;
int lasttime;

uint8_t cnt = 0;

// Float byte dizisi arasında dönüşüm yapmak için
union float2binary
{
  float floating;
  uint8_t binary[4];
};

// void anaAlg(float *alt, float *g, float *pitch, float *roll);
// void anaAlgSetup(void);
// float rampaDegerFonksiyonu(void);
// extern "C" void SystemClock_Config();

HardwareSerial SeriPort(PIN_UART2_RX, PIN_UART2_TX);
ReefwingAHRS ahrs;

long prevTime, prevTime2;
long Time;
bool check = 0;
float maxTotal = 4000;

void setup()
{
  SeriPort.begin(115200);
  DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
  //I2CBegin(PIN_I2C1_SDA, PIN_I2C1_SCL);
  I2CBegin(PB7, PB6); // Bluepill
  DEBUG_PRINTLN(F("I2C Port Baslatildi..."));
  pinMode(LED_BUILTIN, OUTPUT);
  BNO_STR_REGISTERS bnoAyar;
  bnoAyar.registersPage_0.operationMode.operationMode = OPERATIONMODE_AMG;
  bnoAyar.registersPage_1.accConfig.accBandwidth = ACC_BANDWIDTH_500hz;
  BNOBegin(bnoAyar);
  DEBUG_PRINTLN(F("BNO Baslatildi..."));
  ahrs.begin();
  DEBUG_PRINTLN(F("AHRS Filtresi Baslatildi..."));

  ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
  ahrs.setDOF(DOF::DOF_9);

  //BMPInit(BMP_OverSampling_8x, BMP_OverSampling_2x, BMP_IIR_OFF, BMP_ODR_40ms);
  //SystemClock_Config();
  //struct ConfigRF rfayarlari;
  //RFBegin(&rfayarlari, 0x03, 0x05, 23U, UARTPARITY_8N1, UARTBAUDRATE_9600, AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
  managedDelay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  prevTime2 = millis();
  prevTime = millis();
  //uint8_t teeee;
  //I2CReadByte(CHIP_ADR, 0x00, &teeee, TIMEOUT_I2C);

  //DEBUG_PRINT(F("CHIP ID: "));
  //DEBUG_PRINTLN(teeee, HEX);
  //DEBUG_PRINTLN(F("Gyro Kalibrasyon Başladi..."));
  //BNO_GyroCalibration(1000);
  //DEBUG_PRINTLN(F("Gyro Kalibrasyon Bitti..."));
}

void loop()
{
  float gravityX, gravityY, gravityZ;
  float roll, pitch, yaw;
  BNO_DOF3_Float accData, gyroData, magData;
  SensorData imuData;
  uint8_t tt;
  uint8_t buffersend[13];
  uint8_t crc;
  int16_t temp;
  float tempFloat;
  float2binary converter;
  //uint32_t tempraw, presraw;
  //float press, altitude, temp;
  if(millis() > prevTime2 + 2000){
    prevTime2 = millis();
    if(check){
      digitalWrite(LED_BUILTIN, HIGH);
      check = 0;
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
      check = 1;
    }
  }

  accData = getAccData();
  gyroData = getGyroData();
  magData = getMagData();

  imuData.ax = accData.x / 9.81;
  imuData.ay = accData.y / 9.81;
  imuData.az = accData.z / 9.81;

  imuData.gx = gyroData.x;
  imuData.gy = gyroData.y;
  imuData.gz = gyroData.z;

  imuData.mx = magData.x * 0.01;
  imuData.my = magData.y * 0.01;
  imuData.mz = magData.z * 0.01;

  ahrs.setData(imuData);
  ahrs.update();
  
  /*float total = abs(accData.x) + abs(accData.y) + abs(accData.z);
  if(total < maxTotal){
    maxTotal = total;
  }*/

  //motionCalRawPrint(accData, gyroData, magData);

  /*DEBUG_PRINT(accData.x);DEBUG_PRINT(F("   "));
  DEBUG_PRINT(accData.y);DEBUG_PRINT(F("   "));
  DEBUG_PRINTLN(accData.z);
  DEBUG_PRINT(gyroData.x);DEBUG_PRINT(F("   "));
  DEBUG_PRINT(gyroData.y);DEBUG_PRINT(F("   "));
  DEBUG_PRINTLN(gyroData.z);
  DEBUG_PRINT(magData.x);DEBUG_PRINT(F("   "));
  DEBUG_PRINT(magData.y);DEBUG_PRINT(F("   "));
  DEBUG_PRINTLN(magData.z);
  DEBUG_PRINTLN();DEBUG_PRINTLN();DEBUG_PRINTLN();*/

  /*DEBUG_PRINT(F(">AccX:"));DEBUG_PRINTLN(accData.x);
  DEBUG_PRINT(F(">AccY:"));DEBUG_PRINTLN(accData.y);
  DEBUG_PRINT(F(">AccZ:"));DEBUG_PRINTLN(accData.z);
  DEBUG_PRINT(F(">Total:"));DEBUG_PRINTLN(total);
  DEBUG_PRINT(F(">GyroX:"));DEBUG_PRINTLN(gyroData.x);
  DEBUG_PRINT(F(">GyroY:"));DEBUG_PRINTLN(gyroData.y);
  DEBUG_PRINT(F(">GyroZ:"));DEBUG_PRINTLN(gyroData.z);
  DEBUG_PRINT(F(">MagX:"));DEBUG_PRINTLN(magData.x);
  DEBUG_PRINT(F(">MagY:"));DEBUG_PRINTLN(magData.y);
  DEBUG_PRINT(F(">MagZ:"));DEBUG_PRINTLN(magData.z);
  DEBUG_PRINT(F(">MinTotal:"));DEBUG_PRINTLN(maxTotal);*/

  DEBUG_PRINT(F(">AccX:"));DEBUG_PRINTLN(accData.x);
  DEBUG_PRINT(F(">AccY:"));DEBUG_PRINTLN(accData.y);
  DEBUG_PRINT(F(">AccZ:"));DEBUG_PRINTLN(accData.z);
  DEBUG_PRINT(F(">Pitch:"));DEBUG_PRINTLN(90-ahrs.angles.pitch);
  DEBUG_PRINT(F(">Roll:"));DEBUG_PRINTLN(ahrs.angles.roll);
  DEBUG_PRINT(F(">Yaw:"));DEBUG_PRINTLN(ahrs.angles.yaw);

  /*gyroff.x *= RADIAN2DEGREE;
  gyroff.y *= RADIAN2DEGREE;
  gyroff.z *= RADIAN2DEGREE;*/

  /*BMPGetData(&temp, &press, &altitude);

  DEBUG_PRINT(F("Sicaklik: "));
  DEBUG_PRINT(temp);
  DEBUG_PRINT(F("   Basinc: "));
  DEBUG_PRINT(press);
  DEBUG_PRINT(F("   Yukseklik: "));
  DEBUG_PRINTLN(altitude);*/

  /*DEBUG_PRINT(F(">Pitch:")); DEBUG_PRINTLN(pitch);// DEBUG_PRINT(F("    "));
  DEBUG_PRINT(F(">Roll:")); DEBUG_PRINTLN(roll);// DEBUG_PRINT(F("    "));
  DEBUG_PRINT(F(">Yaw:")); DEBUG_PRINTLN(yaw);// DEBUG_PRINT(F("    "));
  DEBUG_PRINT(F(">X:")); DEBUG_PRINTLN(gravityX);// DEBUG_PRINT(F("    "));
  DEBUG_PRINT(F(">Y:")); DEBUG_PRINTLN(gravityY);// DEBUG_PRINT(F("    "));
  DEBUG_PRINT(F(">Z:")); DEBUG_PRINTLN(gravityZ);*/

  //motionCalRawPrint(rawAcc, rawGyro, magData);

  /*DEBUG_PRINT("Counter:");
  DEBUG_PRINTLN(cnt++);*/

  /*DEBUG_PRINTLN(F("MPU Acc Data"));
  DEBUG_PRINT(F(">X:"));
  DEBUG_PRINTLN(accff.x);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(">Y:"));
  DEBUG_PRINTLN(accff.y);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(">Z:"));
  DEBUG_PRINTLN(accff.z);
  DEBUG_PRINTLN( );

  DEBUG_PRINTLN(F("MPU Gyro Data"));
  DEBUG_PRINT(F(" X:"));
  DEBUG_PRINT(gyroff.x);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(" Y:"));
  DEBUG_PRINT(gyroff.y);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(" Z:"));
  DEBUG_PRINTLN(gyroff.z);
  DEBUG_PRINTLN( );

  DEBUG_PRINTLN(F("MAG Data"));
  DEBUG_PRINT(F(" X:"));
  DEBUG_PRINT(magData.x);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(" Y:"));
  DEBUG_PRINT(magData.y);
  DEBUG_PRINT(F("   "));
  DEBUG_PRINT(F(" Z:"));
  DEBUG_PRINTLN(magData.z);
  DEBUG_PRINTLN( );*/

  /*converter.floating = press;
  for(int i = 0; i < 4; i++){
    buffersend[i] = converter.binary[i];
  }
  converter.floating = temp;
  for(int i = 0; i < 4; i++){
    buffersend[i+4] = converter.binary[i];
  }
  converter.floating = altitude;
  for(int i = 0; i < 4; i++){
    buffersend[i+8] = converter.binary[i];
  }
  crc = calculateCRC8(buffersend, 12);
  buffersend[12] = crc;
  DEBUG_PRINT(F("CRC: "));
  DEBUG_PRINTLN(crc);

  sendFixedDataPacket(0x03, 0x05, 23U, buffersend, sizeof(buffersend));
  managedDelay(500);*/

  /*Status res = receiveDataPacket(buffersend, sizeof(buffersend));
  for(int i = 0; i < 4; i++){
    converter.binary[i] = buffersend[i];
  }
  press = converter.floating;
  for(int i = 0; i < 4; i++){
    converter.binary[i] = buffersend[i+4];
  }
  temp = converter.floating;
  for(int i = 0; i < 4; i++){
    converter.binary[i] = buffersend[i+8];
  }
  altitude = converter.floating;
  crc = buffersend[12];
  if(res != E32_Timeout){
    if(crc == calculateCRC8(buffersend, 12)){
      DEBUG_PRINTLN(F("Data: "));
      DEBUG_PRINT(F(" Sicaklik: "));
      DEBUG_PRINT(temp);
      DEBUG_PRINT(F("   Basinc: "));
      DEBUG_PRINT(press);
      DEBUG_PRINT(F("   Yukseklik: "));
      DEBUG_PRINTLN(altitude);
      DEBUG_PRINTLN();
    }
    else{
      DEBUG_PRINTLN(F("CRC Uyusmuyor..."));
    }
  }
  managedDelay(500);*/
  
  managedDelay(40);
}








// Clock Ayarlama
/*extern "C" void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}*/

/*
float rampaDegerFonksiyonu(void)
{
  int32_t toplam = 0, pres, temp;
  float alt, ort;
  for (int i = 0; i < 5; i++)
  {
    alt = getAltitudeReal(&sicaklik, &basinc); // GetAltitudeReal irtifa değerinin döndüren fonksiyondur
    toplam += alt;
    delay(50);
  }
  ort = toplam / (5 * 1.0);
  return ort;
}

void anaAlgSetup(void)
{
  rampaDegerIrtifa = rampaDegerFonksiyonu();
}

void anaAlg(float *alt, float g, float *pitch, float *roll)
{
  if (RampaDeger == true)
  {
    if (*alt > rampaDegerIrtifa + 400)
    {
      RampaDeger = false;
      Serial2.println("Rampa Deger Degeri false oldu...");
      Serial2.println("Rampa Deger Degeri false oldu...");
      Serial2.println("Rampa Deger Degeri false oldu...");
    }
  }

  else
  {
    if (IrtifaSinir == false)
    {
      if (*alt > rampaDegerIrtifa + 600)
      {
        IrtifaSinir = true;
        Serial2.println("Irtifa Sinir Degeri true oldu...");
        Serial2.println("Irtifa Sinir Degeri true oldu...");
        Serial2.println("Irtifa Sinir Degeri true oldu...");
      }
    }

    else
    {
      if (KurtarmaSistemi == false)
      {
        if (bufort - 200 > altitude && ((*pitch > 75 || *pitch < -75) || (*roll > 75 || *roll < -75)))
        {
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          Serial2.println("Kurtarma Sistemi Ateslendi...");
          if (KurtarmaSistemi == false)
          {
            lasttime = millis();
          }

          KurtarmaSistemi = true;
        }

        buftoplam = 0;

        if (buffcnt == 20)
        {
          buffcnt = 0;
        }

        altbuf[buffcnt++] = altitude;

        for (int i = 0; i < 20; i++)
        {
          buftoplam += altbuf[i];
        }

        bufort = buftoplam / 20.0;
      }
    }
  }
}

uint8_t *paketGonderme(){
  uint8_t paket[55];

  
}
*/

void motionCalRawPrint(BNO_DOF3_Float accData, BNO_DOF3_Float gyroData, BNO_DOF3_Float magData){
  accData.x = map(accData.x, 0, 156.9064, 0, 8192);
  accData.y = map(accData.y, 0, 156.9064, 0, 8192);
  accData.z = map(accData.z, 0, 156.9064, 0, 8192);
  DEBUG_PRINT(F("Raw:"));
  DEBUG_PRINT(int(accData.x)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(accData.y)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(accData.z)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(gyroData.x)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(gyroData.y)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(gyroData.z)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(magData.x*10)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(magData.y*10)); DEBUG_PRINT(F(","));
  DEBUG_PRINTLN(int(magData.z*10));
}