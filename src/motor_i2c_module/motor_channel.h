#pragma once

// =============================================================================
// motor_channel.h - один мотор: режимы, выход, PID, защиты
// =============================================================================
//
// Режимы (регистр MODE):
//   Coast — выбег (LOW/LOW), Brake — тормоз (HIGH/HIGH),
//   Pwm   — заданная скважность без обратной связи, с плавным разгоном,
//   Speed — удержание скорости выходного вала по PID.
//
// Любой выход проходит через общий выходной каскад:
//   - предел ±maxOutput;
//   - смена направления только через ноль и паузу REVERSAL_PAUSE_MS;
//   - в режиме Pwm — разгон не быстрее OPEN_LOOP_SLEW_PER_SECOND.
//
// Защиты режима Speed (заклинивание, вращение не в ту сторону) переводят
// мотор в Brake и защёлкивают флаг ошибки до команды CLEAR_ERRORS.
//
// Порядок использования: begin() один раз, step() раз в период контура,
// сеттеры — из обработки команд I2C. Значения проверяет слой регистров,
// сеттеры только применяют их.

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"
#include "quadrature_encoder.h"
#include "speed_meter.h"
#include "zk5ad_driver.h"

enum class MotorMode : uint8_t
{
  Coast = 0,
  Brake = 1,
  Pwm = 2,
  Speed = 3,
};

// Биты регистра FLAGS мотора.
namespace motor_flag
{
constexpr uint8_t REVERSING = 1 << 0;       // выход удерживается в 0 перед реверсом
constexpr uint8_t SATURATED = 1 << 1;       // выход упёрся в MAX_OUTPUT
constexpr uint8_t FAULT_STALL = 1 << 2;     // защёлка: заклинивание
constexpr uint8_t FAULT_DIRECTION = 1 << 3; // защёлка: вращение не в ту сторону
constexpr uint8_t ENCODER_GLITCH = 1 << 4;  // защёлка: недопустимые переходы энкодера
constexpr uint8_t FAULT_MASK = FAULT_STALL | FAULT_DIRECTION;
} // namespace motor_flag

// Биты регистра CONFIG мотора.
namespace motor_config
{
constexpr uint8_t INVERT_MOTOR = 1 << 0;
constexpr uint8_t INVERT_ENCODER = 1 << 1;
constexpr uint8_t MASK = INVERT_MOTOR | INVERT_ENCODER;
} // namespace motor_config

enum class MotorEvent : uint8_t
{
  None,
  Stall,
  WrongDirection,
};

class MotorChannel
{
public:
  void begin(const MotorPins &pins, uint8_t index);

  // Один шаг контура: энкодер, скорость, регулятор, выход, защиты.
  MotorEvent step(uint32_t nowMs, float dtSeconds);

  void setMode(MotorMode mode);
  void setPwmTarget(int16_t output);
  void setSpeedTarget(int16_t deciRpm);
  void setMaxOutput(uint16_t maxOutput);
  void setKp(uint16_t centi);
  void setKi(uint16_t centi);
  void setKd(uint16_t centi);
  void setConfig(uint8_t config);
  void setEncoderCount(int32_t count);

  // Немедленный тормоз (аварийная остановка, watchdog связи).
  void emergencyBrake();
  // Сбрасывает защёлки ошибок. Режим не меняется.
  void clearFaults();

  bool faulted() const { return (flags_ & motor_flag::FAULT_MASK) != 0; }
  bool active() const { return mode_ == MotorMode::Pwm || mode_ == MotorMode::Speed; }

  MotorMode mode() const { return mode_; }
  uint8_t flags() const { return flags_; }
  uint8_t config() const { return config_; }
  int16_t pwmTarget() const { return pwmTarget_; }
  int16_t speedTarget() const { return speedTarget_; }
  int16_t speedDeciRpm() const;
  float rpm() const { return speed_.rpm(); }
  int16_t output() const { return applied_; }
  int32_t encoderCount() const;
  uint16_t maxOutput() const { return maxOutput_; }
  uint16_t kp() const { return kp_; }
  uint16_t ki() const { return ki_; }
  uint16_t kd() const { return kd_; }
  uint32_t encoderGlitches() const { return encoder_.glitches(); }

private:
  void applyStop(MotorMode mode);
  void restartPid();
  void applyTunings();
  void driveTowards(int16_t target, bool slew, uint32_t nowMs, float dtSeconds);
  MotorEvent checkProtections(uint32_t nowMs);
  void latchFault(uint8_t flag);

  Zk5adChannel driver_;
  QuadratureEncoder encoder_;
  SpeedMeter speed_;
  PidController pid_;

  MotorMode mode_ = MotorMode::Coast;
  uint8_t config_ = 0;
  uint8_t flags_ = 0;
  int16_t pwmTarget_ = 0;    // промилле
  int16_t speedTarget_ = 0;  // 0.1 об/мин
  uint16_t maxOutput_ = DEFAULT_MAX_OUTPUT;
  uint16_t kp_ = 0;          // сотые
  uint16_t ki_ = 0;
  uint16_t kd_ = 0;

  // Выходной каскад.
  int16_t applied_ = 0;       // текущий выход, промилле
  int8_t lastSign_ = 0;       // направление последнего ненулевого выхода
  uint32_t zeroSinceMs_ = 0;  // когда выход стал нулевым
  float slewRemainder_ = 0.0f;

  // Защиты.
  bool stallTiming_ = false;
  uint32_t stallSinceMs_ = 0;
  bool wrongDirTiming_ = false;
  uint32_t wrongDirSinceMs_ = 0;
  float wrongDirRpm_ = 0.0f;

  uint32_t glitchBaseline_ = 0;
};
