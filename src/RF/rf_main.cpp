#include <Arduino.h>
#include <EByte_LoRa_E32_library.h>

void setsMainOpt();
void rfInit();

LoRa_E32 e32ttl100(&Serial2);

void rfInit(){
    
    e32ttl100.begin();
}

void setsMainOpt(){
    // Bu Function RF'in başlangıçtaki temel ayarlarını yapmaktadır.

    ResponseStructContainer ayar;
    ayar = e32ttl100.getConfiguration();

    Configuration conf = *(Configuration*) ayar.data;

    conf.ADDL = 0x0;
    conf.ADDH = 0x1;
    conf.CHAN = 0x19;

    conf.OPTION.fec = FEC_1_ON;
    conf.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    conf.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;


    conf.SPED.airDataRate = AIR_DATA_RATE_011_48;
    conf.SPED.uartBaudRate = UART_BPS_9600;
    conf.SPED.uartParity = MODE_00_8N1;



}