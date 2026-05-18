#ifndef _WISCORE_RAK4631_H_
#define _WISCORE_RAK4631_H_

#include <stdint.h>

#define PINS_COUNT           48
#define NUM_DIGITAL_PINS     48
#define NUM_ANALOG_INPUTS    6

// LED Konfiguration für den nRF52-Core
#define LED_STATE_ON         0  // 0 = LOW (Active Low)
#define LED_STATE_OFF        1

// SPI Hardware Pins
#define PIN_SPI_MISO         45
#define PIN_SPI_MOSI         44
#define PIN_SPI_SCK          43
#define SS                   42

#define SPI_INTERFACES_COUNT 1

// UART / Serial1 Pins für RS485 (RAK5802 Slot A: WB_IO4=P0.15, WB_IO5=P0.16)
#define PIN_SERIAL1_RX       15
#define PIN_SERIAL1_TX       16

// WisBlock IO Definitions
#define WB_IO1               17
#define WB_IO2               34

// WisBlock Analog Pins
#define WB_A0                5    // P0.05 / AIN3 — Batteriespannung (Spannungsteiler)

// LoRa SX1262 Pins
#define LORA_RESET      38  // Statt PIN_LORA_RESET
#define LORA_NSS        42  // Statt PIN_LORA_NSS
#define LORA_SCLK       43  
#define LORA_MOSI       44
#define LORA_MISO       45
#define LORA_BUSY       46

#define LED_BLUE             35
#define LED_GREEN            36

#if !defined(NRF_SPIM_Type)
  static const uint8_t MOSI = PIN_SPI_MOSI;
  static const uint8_t MISO = PIN_SPI_MISO;
  static const uint8_t SCK  = PIN_SPI_SCK;
#endif

#endif