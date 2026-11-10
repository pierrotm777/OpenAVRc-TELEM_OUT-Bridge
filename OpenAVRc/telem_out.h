/*
 * OpenAVRc - Serial1 telemetry output
 */
#ifndef TELEM_OUT_H
#define TELEM_OUT_H

#include <stdint.h>

void telemOutInit();
void telemOutStop();
void telemOutMain();
void telemOutSend(const uint8_t *data, uint8_t len);

#endif
