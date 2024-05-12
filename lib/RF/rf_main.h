#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

void setsMainOpt(void);
void setsMainOptreceiver();
void rfInitReceiver();
void rfInit(void);
void sendMessage(void);
void printParameters(struct Configuration conf);
void haberlesmeTestTransmitter(void);
void haberlesmeTestReceiver(void);
void denemeHaberlesmeReceiver();
void denemeHaberlesmeTransmitter();
void testdenemeTransmitter(float *irtifa, int32_t *sicaklik, int32_t *basinc, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *aci, float *enlem, float *boylam);
void testdenemeReceiver();
void denemeddTransmitter(float *irtifa);
void denemeddReceiver(void);