#include <Arduino.h>

// DYNAMIXEL TTL Comms pins
const uint8_t DXL_DIR_PIN = 5;

// Chip LED
const uint8_t LED_PIN = 4;

// RGB LED
const uint8_t RGB_PIN = 0;

// Buck converter enables (see PowerRail) - opposite polarities. The V3.0.2
// PCB used these same two pins as the power button sense (IO1) and power
// latch (IO10); that circuit is gone.
const uint8_t BUCK_5V_EN_PIN = 10;  // IO10: HIGH = 5V buck on (peripherals, e.g. the RGB LED)
const uint8_t BUCK_5V_ON     = HIGH;
const uint8_t BUCK_6V_EN_PIN = 1;   // IO1: LOW = 6V buck on (servos)
const uint8_t BUCK_6V_ON     = LOW;

// Pairing button (new PCB revision) - active low, long-press forces a re-pair
const uint8_t BUTTON_PIN = 9;