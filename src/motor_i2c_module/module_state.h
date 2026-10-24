#pragma once

// =============================================================================
// module_state.h - состояние модуля: моторы, защёлки ошибок, watchdog связи
// =============================================================================
//
// Блокирующие защёлки (ESTOP, WATCHDOG, ошибка мотора) запрещают переводить
// моторы в PWM / SPEED, пока master не отправит CLEAR_ERRORS. Так модуль не
// поедет сам после восстановления связи или устранения ошибки.

#include <stdint.h>
#include "config.h"
#include "motor_channel.h"
#include "register_map.h"

// Флаги причины последнего сброса MCU (регистр RESET_FLAGS).
namespace reset_flag
{
constexpr uint8_t POWER_ON = 1 << 0;
constexpr uint8_t PIN = 1 << 1;      // кнопка RESET или NRST
constexpr uint8_t SOFTWARE = 1 << 2; // в том числе перезагрузка из загрузчика
constexpr uint8_t WATCHDOG_RESET = 1 << 3; // IWDG: завис loop()
} // namespace reset_flag

struct ModuleState
{
  MotorChannel motors[MOTOR_COUNT];

  uint8_t latches = 0; // биты status::*, кроме MOTOR_FAULT
  uint8_t lastError = error::NONE;
  uint8_t stopReason = stop_reason::NONE;
  uint8_t stopMotor = 0xFF;
  uint8_t resetFlags = 0;
  uint16_t watchdogMs = COMM_WATCHDOG_DEFAULT_MS;
  uint16_t framesOk = 0;
  uint16_t framesRejected = 0;

  // Значение регистра STATUS.
  uint8_t status() const;
  bool anyMotorFaulted() const;
  bool anyMotorActive() const;

  // Тормоз всех моторов и запись причины. ESTOP и WATCHDOG защёлкиваются.
  void stopAll(uint8_t reason);
  // Ошибка защиты одного мотора (он уже затормозил сам).
  void motorFault(uint8_t motor, uint8_t reason);
  // Сброс всех защёлок, LAST_ERROR и STOP_REASON.
  void clearErrors();
  // Запись отклонена: LAST_ERROR, бит STATUS и счётчик.
  void rejectFrame(uint8_t code);

  // Watchdog связи: вызывать на каждом проходе loop().
  void checkCommWatchdog(uint32_t lastContactMs);
};
