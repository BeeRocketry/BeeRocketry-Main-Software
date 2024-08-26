#ifndef GPS__H
#define GPS__H

#include <Arduino.h>
#include "debugprinter.h"
#include <TinyGPSPlus.h>

extern HardwareSerial GPSSeriPort;

static void smartDelay(unsigned long ms);
void getGPSData(float *enlem, float *boylam, float *irtifa, int16_t *satellite);

#endif