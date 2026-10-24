#pragma once

// =============================================================================
// zk5ad_driver.h - один канал драйвера ZK-5AD на двух каналах таймера
// =============================================================================
//
// У TA6586 нет входа разрешения: скорость задаётся ШИМ на одном входе, пока
// на другом LOW.
//
//   IN1   IN2   результат
//   PWM   LOW   вперёд, ШИМ «вращение/выбег»
//   LOW   PWM   назад
//   LOW   LOW   выбег (выходы отключены)
//   HIGH  HIGH  тормоз (выводы мотора замкнуты)
//
// Оба входа постоянно работают как каналы ШИМ (PWM1): CCR = 0 даёт
// постоянный LOW, CCR = ARR + 1 — постоянный HIGH. Поэтому выбег, тормоз и
// вращение переключаются только записью CCR, без смены режима пинов.

#include <Arduino.h>
#include "config.h"

class Zk5adChannel
{
public:
  // Оба входа сначала в LOW как обычные выходы, затем передаются таймеру с
  // нулевой скважностью. Мотор не дёргается ни на одном шаге.
  void begin(const MotorPins &pins, bool invert);

  // Запускает таймеры ШИМ. Вызывается один раз после begin() всех каналов.
  static void startTimers();

  // output: -1000..1000 промилле; 0 — выбег.
  void drive(int16_t output);
  void coast();
  void brake();

  void setInvert(bool invert) { invert_ = invert; }

private:
  void write(uint32_t in1Ticks, uint32_t in2Ticks);

  HardwareTimer *timer_ = nullptr;
  uint8_t in1Channel_ = 0;
  uint8_t in2Channel_ = 0;
  uint32_t periodTicks_ = 0; // ARR + 1
  bool invert_ = false;
};
