#include <Arduino.h>

// Встроенный светодиод Blue Pill: PC13, активен низким уровнем (LOW = горит)
const uint32_t LED_PIN = PC13;
const uint8_t LED_ON_LEVEL = LOW;
const uint8_t LED_OFF_LEVEL = HIGH;

// Полупериод мигания: 500 мс горит / 500 мс не горит
const uint32_t BLINK_HALF_PERIOD_MS = 300;

bool ledOn = false;
uint32_t lastToggleMs = 0;

void setLed(bool on) {
  digitalWrite(LED_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
  Serial.println(on ? "LED ON" : "LED OFF");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  setLed(ledOn);
  lastToggleMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastToggleMs >= BLINK_HALF_PERIOD_MS) {
    lastToggleMs += BLINK_HALF_PERIOD_MS;
    ledOn = !ledOn;
    setLed(ledOn);
  }
}
