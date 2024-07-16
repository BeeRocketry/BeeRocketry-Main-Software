#include <Arduino.h>

#define DEBUG_MODE

#include <HardwareSerial.h>
#include "main.h"
#include "rf.h"
#include "bmp388.h"
//#include "MPU.h"

// Ana algoritma devresi için genel obje tanımlamaları değişken tanımları
bool RampaDeger = true;
bool IrtifaSinir = false;
bool KurtarmaSistemi = false;
float rampaDegerIrtifa = 0;
float altitude;
int32_t sicaklik, basinc;
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

HardwareSerial SeriPort(PA3, PA2);

void setup()
{
  SeriPort.begin(9600);
  DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
  //SystemClock_Config();
  struct ConfigRF rfayarlari;
  RFBegin(&rfayarlari, 0x01, 0x06, 23U, UARTPARITY_8N1, UARTBAUDRATE_9600, AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
  delay(1000);
}

void loop()
{
  /*delay(50);
  altitude = getAltitudeReal(&sicaklik, &basinc);
  Serial2.print("Irtifa: ");
  Serial2.println(altitude);
  Serial2.println();*/

  /*sendFixedSingleData(0x01, 0x06, 23U, cnt);
  DEBUG_PRINT(F("Cnt: "));
  DEBUG_PRINTLN(cnt);
  cnt += 3;
  managedDelay(500);*/

  uint8_t data;
  Status res = receiveSingleData(&data);
  if(res != E32_Timeout){
    DEBUG_PRINT(F("Data: "));
    DEBUG_PRINTLN(data);
  }
  managedDelay(500);
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