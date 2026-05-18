#include <Arduino.h>
#include <stdint.h>

// Das Array, das dem Linker fehlt. 
// Es bildet die Arduino-Pin-Nummern (0-47) auf die nRF52-Ports ab.
const uint32_t g_ADigitalPinMap[] = {
  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 
  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47
};

// Falls das Framework noch weitere Variablen erwartet:
const uint8_t g_APinDescription[] = {0};