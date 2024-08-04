#ifndef GPS__H
#define GPS__H

#include <Arduino.h>
#include "debugprinter.h"
#include <TinyGPS.h>

extern HardwareSerial GPSSeriPort;

static void smartdelay(unsigned long ms);
void getGPSData(float *enlem, float *boylam, float *irtifa = (float *)0, float *hiz = (float *)0);
void gpsDatas(float *enlem, float *boylam, int *irtifa);

#endif