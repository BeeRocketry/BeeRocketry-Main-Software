#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

void setsMainOpt();
void rfInit();

LoRa_E32 e32ttl1w(&Serial2);

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
    conf.ADDH = 0x1;
    conf.CHAN = 0x19;

    conf.OPTION.fec = FEC_1_ON;
    conf.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    conf.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    conf.OPTION.wirelessWakeupTime = WAKE_UP_250;
    conf.OPTION.transmissionPower = POWER_30;

    conf.SPED.airDataRate = AIR_DATA_RATE_011_48;
    conf.SPED.uartBaudRate = UART_BPS_9600;
    conf.SPED.uartParity = MODE_00_8N1;

    ResponseStatus rs = e32ttl1w.setConfiguration(conf, WRITE_CFG_PWR_DWN_LOSE);
    ayar.close();
}