#include "module_state.h"

#include <Arduino.h>
#include "debug_log.h"

uint8_t ModuleState::status() const
{
  return latches | (anyMotorFaulted() ? status::MOTOR_FAULT : 0);
}

bool ModuleState::anyMotorFaulted() const
{
  for (const MotorChannel &motor : motors)
  {
    if (motor.faulted())
    {
      return true;
    }
  }
  return false;
}

bool ModuleState::anyMotorActive() const
{
  for (const MotorChannel &motor : motors)
  {
    if (motor.active())
    {
      return true;
    }
  }
  return false;
}

void ModuleState::stopAll(uint8_t reason)
{
  for (MotorChannel &motor : motors)
  {
    motor.emergencyBrake();
  }
  stopReason = reason;
  stopMotor = 0xFF;
  if (reason == stop_reason::ESTOP)
  {
    latches |= status::ESTOP;
    debugLog("ESTOP: all motors braked\r\n");
  }
  else if (reason == stop_reason::WATCHDOG)
  {
    latches |= status::WATCHDOG;
    debugLog("WATCHDOG: no I2C for %u ms, all motors braked\r\n", watchdogMs);
  }
}

void ModuleState::motorFault(uint8_t motor, uint8_t reason)
{
  stopReason = reason;
  stopMotor = motor;
  debugLog("FAULT motor %u: %s, braked\r\n", motor + 1,
           reason == stop_reason::STALL ? "stall" : "wrong direction");
}

void ModuleState::clearErrors()
{
  latches = 0;
  lastError = error::NONE;
  stopReason = stop_reason::NONE;
  stopMotor = 0xFF;
  for (MotorChannel &motor : motors)
  {
    motor.clearFaults();
  }
}

void ModuleState::rejectFrame(uint8_t code)
{
  lastError = code;
  framesRejected++;
  switch (code)
  {
  case error::CRC_MISMATCH:
    latches |= status::CRC_ERROR;
    break;
  case error::VALUE:
    latches |= status::VALUE_ERROR;
    break;
  case error::STATE:
    latches |= status::STATE_ERROR;
    break;
  case error::RX_OVERFLOW:
    latches |= status::RX_OVERFLOW;
    break;
  default:
    latches |= status::FRAME_ERROR;
    break;
  }
  debugLog("I2C write rejected, error %u\r\n", code);
}

void ModuleState::checkCommWatchdog(uint32_t lastContactMs)
{
  if (!anyMotorActive())
  {
    return;
  }
  // millis() читается после lastContactMs: прерывание I2C между двумя
  // чтениями сделало бы разность «отрицательной», то есть огромной.
  const uint32_t now = millis();
  if (now - lastContactMs > watchdogMs)
  {
    stopAll(stop_reason::WATCHDOG);
  }
}
