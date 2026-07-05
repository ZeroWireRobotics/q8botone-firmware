#include <Arduino.h>

// DYNAMIXEL TTL Comms pins
const uint8_t DXL_DIR_PIN = 5;

// Chip LED
const uint8_t LED_PIN = 4;

// RGB LED
const uint8_t RGB_PIN = 0;

// Power circuit (V3.0.2 PCB)
const uint8_t SENSE_PIN = 1;   // IO1 = BTN_SEN
const uint8_t LATCH_PIN = 10;  // IO10 = LATCH