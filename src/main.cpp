#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
//#include <TinyGPS++.h>
//#include <SD.h>
#include <SPI.h>
#include "main.h"
#include "rf.h"
#include "bmp388.h"
#include "MMC5603.h"
#include "MPU.h"
#include "ICM20948.h"

void motionCalRawPrint(Dof3Data_Int accData, Dof3Data_Int gyroData, Dof3Data_FloatMMC magData);

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

long prevTime;
bool check = 0;

void setup()
{
  MPU_REGISTERS settingsMPU;
  SeriPort.begin(115200);
  DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
  //I2CBegin(PIN_I2C1_SDA, PIN_I2C1_SCL);
  I2CBegin(PB7, PB6); // Bluepill
  DEBUG_PRINTLN(F("I2C Port Baslatildi..."));
  pinMode(LED_BUILTIN, OUTPUT);
  MMCBegin(true, 1000);
  DEBUG_PRINTLN(F("MMC Port Baslatildi..."));
  mpuInit(&settingsMPU);
  DEBUG_PRINTLN(F("MPU Port Baslatildi..."));
  //BMPInit(BMP_OverSampling_8x, BMP_OverSampling_2x, BMP_IIR_OFF, BMP_ODR_40ms);
  //SystemClock_Config();
  //struct ConfigRF rfayarlari;
  //RFBegin(&rfayarlari, 0x03, 0x05, 23U, UARTPARITY_8N1, UARTBAUDRATE_9600, AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
  managedDelay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  prevTime = millis();
  //uint8_t teeee;
  //I2CReadByte(CHIP_ADR, 0x00, &teeee, TIMEOUT_I2C);

  //DEBUG_PRINT(F("CHIP ID: "));
  //DEBUG_PRINTLN(teeee, HEX);
}

void loop()
{
  Dof3Data_IntMAG rawData;
  Dof3Data_FloatMMC magData;

  Dof3Data_Int rawGyro;
  Dof3Data_Int rawAcc;

  Dof3Data_Float gyroff;
  Dof3Data_Float accff;
  uint8_t tt;
  uint8_t buffersend[13];
  uint8_t crc;
  int16_t temp;
  float tempFloat;
  float2binary converter;
  //uint32_t tempraw, presraw;
  //float press, altitude, temp;
  if(millis() > prevTime + 2000){
    prevTime = millis();
    if(check){
      digitalWrite(LED_BUILTIN, HIGH);
      check = 0;
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
      check = 1;
    }
  }

  getRawAccGyroTempData(&rawAcc, &rawGyro, &temp);
  normalizeAccGyroTempData(&rawAcc, &rawGyro, &temp, &tempFloat, &accff, &gyroff);
  managedDelay(5);

  /*BMPGetData(&temp, &press, &altitude);

  DEBUG_PRINT(F("Sicaklik: "));
  DEBUG_PRINT(temp);
  DEBUG_PRINT(F("   Basinc: "));
  DEBUG_PRINT(press);
  DEBUG_PRINT(F("   Yukseklik: "));
  DEBUG_PRINTLN(altitude);*/
  //getMagData(&rawData, &magData);
  getRawMagData(&rawData);
  getHighCalibrated(&rawData);
  getSoftCalibrateda(&rawData);
  compensatedMagData(rawData, &magData);

  motionCalRawPrint(rawAcc, rawGyro, magData);

  /*DEBUG_PRINT("Counter:");
  DEBUG_PRINTLN(cnt++);

  DEBUG_PRINTLN(F("MPU Acc Data"));
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

void motionCalRawPrint(Dof3Data_Int accData, Dof3Data_Int gyroData, Dof3Data_FloatMMC magData){
  DEBUG_PRINT(F("Raw:"));
  DEBUG_PRINT(accData.x); DEBUG_PRINT(F(","));
  DEBUG_PRINT(accData.y); DEBUG_PRINT(F(","));
  DEBUG_PRINT(accData.z); DEBUG_PRINT(F(","));
  DEBUG_PRINT(gyroData.x); DEBUG_PRINT(F(","));
  DEBUG_PRINT(gyroData.y); DEBUG_PRINT(F(","));
  DEBUG_PRINT(gyroData.z); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(magData.x*10)); DEBUG_PRINT(F(","));
  DEBUG_PRINT(int(magData.y*10)); DEBUG_PRINT(F(","));
  DEBUG_PRINTLN(int(magData.z*10));
}