// =============================================================================
// motor_i2c_module - четыре DC-мотора с энкодерами, управление по I2C
// =============================================================================
//
// Blue Pill (STM32F103C8T6), I2C slave по адресу I2C_ADDRESS на I2C2
// (PB10 SCL, PB11 SDA). Протокол — doc/motor_i2c_module/i2c-protocol.md.
//
// loop() ничего не ждёт:
//   1. разбирает записи, принятые прерыванием I2C;
//   2. раз в CONTROL_PERIOD_MS — шаг контура всех моторов;
//   3. проверяет watchdog связи;
//   4. обновляет снимок регистров для чтения;
//   5. мигает светодиодом и печатает состояние в Serial (USB CDC).

#include <Arduino.h>
#include <IWatchdog.h>

#include "config.h"
#include "debug_log.h"
#include "i2c_link.h"
#include "module_state.h"
#include "registers.h"

namespace
{
ModuleState module;
uint8_t image[reg::MAP_SIZE];
uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;

const char *const MODE_NAMES[] = {"COAST", "BRAKE", "PWM", "SPEED"};

uint8_t readResetFlags()
{
  const uint32_t csr = RCC->CSR;
  uint8_t flags = 0;
  if (csr & RCC_CSR_PORRSTF)
  {
    flags |= reset_flag::POWER_ON;
  }
  if (csr & RCC_CSR_PINRSTF)
  {
    flags |= reset_flag::PIN;
  }
  if (csr & RCC_CSR_SFTRSTF)
  {
    flags |= reset_flag::SOFTWARE;
  }
  if (csr & RCC_CSR_IWDGRSTF)
  {
    flags |= reset_flag::WATCHDOG_RESET;
  }
  RCC->CSR |= RCC_CSR_RMVF; // следующий сброс начнёт с чистых флагов
  return flags;
}

void controlStep(uint32_t nowMs, float dtSeconds)
{
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    switch (module.motors[i].step(nowMs, dtSeconds))
    {
    case MotorEvent::Stall:
      module.motorFault(i, stop_reason::STALL);
      break;
    case MotorEvent::WrongDirection:
      module.motorFault(i, stop_reason::WRONG_DIRECTION);
      break;
    case MotorEvent::None:
      break;
    }
  }
}

// Ровно: 1 Гц — норма, 5 Гц — есть блокирующая ошибка (нужен CLEAR_ERRORS).
void updateLed(uint32_t nowMs)
{
  const bool blocked = (module.status() & status::BLOCKING_MASK) != 0;
  const uint32_t halfPeriod = blocked ? 100 : 500;
  const bool on = (nowMs / halfPeriod) % 2 == 0;
  digitalWrite(LED_PIN, on ? LOW : HIGH); // PC13 активен низким уровнем
}

void printStatus(uint32_t nowMs)
{
  if (nowMs - lastDebugMs < DEBUG_PERIOD_MS)
  {
    return;
  }
  lastDebugMs = nowMs;
  debugLog("t=%lu status=0x%02X err=%u stop=%u ok=%u rej=%u\r\n",
           static_cast<unsigned long>(nowMs), module.status(), module.lastError,
           module.stopReason, module.framesOk, module.framesRejected);
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    const MotorChannel &motor = module.motors[i];
    const int16_t speed = motor.speedDeciRpm();
    const int16_t target = motor.speedTarget();
    debugLog("  m%u %-5s flags=0x%02X out=%5d tgt=%s%d.%d rpm=%s%d.%d cnt=%ld glitch=%lu\r\n",
             i + 1, MODE_NAMES[static_cast<uint8_t>(motor.mode())], motor.flags(),
             motor.output(),
             target < 0 ? "-" : "", abs(target) / 10, abs(target) % 10,
             speed < 0 ? "-" : "", abs(speed) / 10, abs(speed) % 10,
             static_cast<long>(motor.encoderCount()),
             static_cast<unsigned long>(motor.encoderGlitches()));
  }
}
} // namespace

void setup()
{
  // Первым делом — входы ZK-5AD в LOW, до любых задержек и USB.
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    module.motors[i].begin(MOTOR_PINS[i], i);
  }
  Zk5adChannel::startTimers();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  module.resetFlags = readResetFlags();
  Serial.begin(115200); // USB CDC: скорость не влияет

  registers::buildImage(module, image);
  i2c_link::publish(image);
  i2c_link::begin();

  IWatchdog.begin(IWDG_TIMEOUT_US);
  lastControlUs = micros();
  debugLog("motor_i2c_module %u.%u, I2C 0x%02X, reset flags 0x%02X\r\n",
           FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, I2C_ADDRESS, module.resetFlags);
}

void loop()
{
  IWatchdog.reload();
  bool changed = false;

  i2c_link::Frame frame;
  while (i2c_link::popFrame(frame))
  {
    registers::applyFrame(module, frame);
    changed = true;
  }
  const uint16_t dropped = i2c_link::takeDroppedFrames();
  for (uint16_t i = 0; i < dropped; i++)
  {
    module.rejectFrame(error::RX_OVERFLOW);
    changed = true;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastControlUs;
  if (elapsedUs >= CONTROL_PERIOD_MS * 1000UL)
  {
    lastControlUs = nowUs;
    controlStep(millis(), elapsedUs / 1e6f);
    changed = true;
  }

  module.checkCommWatchdog(i2c_link::lastContactMs());

  if (changed)
  {
    registers::buildImage(module, image);
    i2c_link::publish(image);
  }

  const uint32_t nowMs = millis();
  updateLed(nowMs);
  printStatus(nowMs);
}
