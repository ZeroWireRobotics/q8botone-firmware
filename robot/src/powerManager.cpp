#include "powerManager.h"

PowerManager::PowerManager(uint8_t latchPin, uint8_t sensePin, uint8_t ledPin)
  : _latchPin(latchPin), _sensePin(sensePin), _ledPin(ledPin) {}

bool PowerManager::begin() {
  pinMode(_latchPin, OUTPUT);
  pinMode(_sensePin, INPUT_PULLUP);  // No external pull-up on BTN SENSE line
  // LED_PIN direction is set by the caller (main.cpp setup) and shared with
  // other tasks; we don't own it exclusively.

  digitalWrite(_latchPin, LOW);
  analogWrite(_ledPin, 0);

  // Button pressed = SENSE_PIN LOW (active-low via D7 to S1).
  // If button isn't held at boot, don't latch — power dies naturally.
  if (digitalRead(_sensePin) == HIGH) {
    return false;
  }

  // Fade LED in over HOLD_MS while confirming the button is still held.
  uint32_t start = millis();
  while (true) {
    uint32_t elapsed = millis() - start;

    if (digitalRead(_sensePin) == HIGH) {
      // Button released early — abort, LED off, power dies naturally.
      analogWrite(_ledPin, 0);
      return false;
    }

    if (elapsed >= HOLD_MS) {
      break;
    }

    analogWrite(_ledPin, (uint8_t)((elapsed * 255UL) / HOLD_MS));
    delay(POLL_MS);
  }

  // 3 seconds held — latch power on.
  analogWrite(_ledPin, 255);
  digitalWrite(_latchPin, HIGH);
  return true;
}

void PowerManager::powerOff() {
  analogWrite(_ledPin, 0);
  digitalWrite(_latchPin, LOW);
}

void PowerManager::startMonitorTask() {
  xTaskCreate(_monitorTask, "PowerMonitor", 2048, this, 2, NULL);
}

void PowerManager::_monitorTask(void* param) {
  PowerManager* self = static_cast<PowerManager*>(param);

  while (true) {
    // Wait for button press (SENSE_PIN LOW = pressed).
    if (digitalRead(self->_sensePin) == HIGH) {
      vTaskDelay(pdMS_TO_TICKS(self->POLL_MS));
      continue;
    }

    // Button is now held — fade LED out over HOLD_MS.
    uint32_t pressStart = millis();
    bool aborted = false;

    while (true) {
      uint32_t elapsed = millis() - pressStart;

      if (digitalRead(self->_sensePin) == HIGH) {
        // Released early — abort shutdown, restore LED.
        analogWrite(self->_ledPin, 255);
        aborted = true;
        break;
      }

      if (elapsed >= HOLD_MS) {
        break;
      }

      analogWrite(self->_ledPin, (uint8_t)(255 - (elapsed * 255UL) / HOLD_MS));
      vTaskDelay(pdMS_TO_TICKS(self->POLL_MS));
    }

    if (!aborted) {
      // Hold confirmed — cut power.
      analogWrite(self->_ledPin, 0);
      digitalWrite(self->_latchPin, LOW);
      vTaskDelete(NULL);
    }
  }
}
