#include "rf_main.h"
#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

#define AUX_PIN 1

void setsMainOpt(void);
void rfInit(void);

HardwareSerial UART2(PA3, PA2);
LoRa_E32 e32ttl1w(&UART2, AUX_PIN);

void rfInit(){
    e32ttl1w.begin();

    setsMainOpt();
}

void setsMainOpt(){
    // Bu Function RF'in başlangıçtaki temel ayarlarını yapmaktadır.

    ResponseStructContainer ayar;
    ayar = e32ttl1w.getConfiguration();

    Configuration conf = *(Configuration*) ayar.data;

    conf.ADDL = 0x0;
    conf.ADDH = 0x6;
    conf.CHAN = 0x759;

    conf.OPTION.fec = FEC_1_ON;
    conf.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    conf.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    conf.OPTION.wirelessWakeupTime = WAKE_UP_250;
    conf.OPTION.transmissionPower = POWER_30;

    conf.SPED.airDataRate = AIR_DATA_RATE_011_48;
    conf.SPED.uartBaudRate = UART_BPS_9600;
    conf.SPED.uartParity = MODE_00_8N1;

    ResponseStatus rs = e32ttl1w.setConfiguration(conf, WRITE_CFG_PWR_DWN_SAVE);

    printParameters(conf);
    ayar.close();
}

void printParameters(struct Configuration conf){
    // Bu Fonksiyon RF modülünün tüm ayarlarını printliyor.

    Serial.println("----------------------------------------");

    Serial.print(F("HEAD : "));  Serial.print(conf.HEAD, BIN);Serial.print(" ");Serial.print(conf.HEAD, DEC);Serial.print(" ");Serial.println(conf.HEAD, HEX);
    Serial.println(F(" "));
    Serial.print(F("AddH : "));  Serial.println(conf.ADDH, BIN);
    Serial.print(F("AddL : "));  Serial.println(conf.ADDL, BIN);
    Serial.print(F("Chan : "));  Serial.print(conf.CHAN, DEC); Serial.print(" -> "); Serial.println(conf.getChannelDescription());
    Serial.println(F(" "));
    Serial.print(F("SpeedParityBit     : "));  Serial.print(conf.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(conf.SPED.getUARTParityDescription());
    Serial.print(F("SpeedUARTDatte  : "));  Serial.print(conf.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(conf.SPED.getUARTBaudRate());
    Serial.print(F("SpeedAirDataRate   : "));  Serial.print(conf.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(conf.SPED.getAirDataRate());
 
    Serial.print(F("OptionTrans        : "));  Serial.print(conf.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(conf.OPTION.getFixedTransmissionDescription());
    Serial.print(F("OptionPullup       : "));  Serial.print(conf.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(conf.OPTION.getIODroveModeDescription());
    Serial.print(F("OptionWakeup       : "));  Serial.print(conf.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(conf.OPTION.getWirelessWakeUPTimeDescription());
    Serial.print(F("OptionFEC          : "));  Serial.print(conf.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(conf.OPTION.getFECDescription());
    Serial.print(F("OptionPower        : "));  Serial.print(conf.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(conf.OPTION.getTransmissionPowerDescription());
 
    Serial.println("----------------------------------------");
}

void sendMessage(void){
    struct Message
    {
        byte altitude[4];
        byte gpsAltitude[4];
        byte gpsEnlem[4];
        byte gpsBoylam[4];
        byte gyroX[4];
        byte gyroY[4];
        byte gyroZ[4];
        byte accX[4];
        byte accY[4];
        byte accZ[4];
    } mes;

    *(float*)(mes.altitude) = Yukseklik; // Örnek atama
    
    ResponseStatus mes = e32ttl1w.sendMessage(&mes, sizeof(mes));
}

void sendFixedMessage(String message, int highadr, int lowadr, int chan){
    ResponseStatus mes = e32ttl1w.sendFixedMessage(highadr, lowadr, chan, message);
}