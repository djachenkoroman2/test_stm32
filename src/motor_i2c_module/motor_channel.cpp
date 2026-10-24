#include "motor_channel.h"

namespace
{
int8_t signOf(int32_t value)
{
  return (value > 0) - (value < 0);
}

uint16_t toCenti(float value)
{
  return static_cast<uint16_t>(lroundf(value * 100.0f));
}
} // namespace

void MotorChannel::begin(const MotorPins &pins, uint8_t index)
{
  config_ = (DEFAULT_INVERT_MOTOR ? motor_config::INVERT_MOTOR : 0) |
            (DEFAULT_INVERT_ENCODER ? motor_config::INVERT_ENCODER : 0);
  driver_.begin(pins, (config_ & motor_config::INVERT_MOTOR) != 0);
  encoder_.begin(pins, index);
  speed_.reset(encoderCount());

  kp_ = toCenti(DEFAULT_KP);
  ki_ = toCenti(DEFAULT_KI);
  kd_ = toCenti(DEFAULT_KD);
  applyTunings();

  mode_ = MotorMode::Coast;
  applied_ = 0;
  lastSign_ = 0;
  zeroSinceMs_ = millis();
  glitchBaseline_ = encoder_.glitches();
}

MotorEvent MotorChannel::step(uint32_t nowMs, float dtSeconds)
{
  encoder_.update();
  speed_.update(encoderCount(), dtSeconds);

  if (encoder_.glitches() != glitchBaseline_)
  {
    flags_ |= motor_flag::ENCODER_GLITCH;
  }

  MotorEvent event = MotorEvent::None;
  switch (mode_)
  {
  case MotorMode::Pwm:
    driveTowards(pwmTarget_, true, nowMs, dtSeconds);
    if (abs(pwmTarget_) > maxOutput_)
    {
      flags_ |= motor_flag::SATURATED;
    }
    break;

  case MotorMode::Speed:
  {
    const float rpm = speed_.rpm();
    float output = 0.0f;
    if (speedTarget_ == 0)
    {
      pid_.reset(rpm);
    }
    else
    {
      output = pid_.update(speedTarget_ / 10.0f, rpm, dtSeconds);
    }
    driveTowards(static_cast<int16_t>(lroundf(output)), false, nowMs, dtSeconds);
    if ((flags_ & motor_flag::REVERSING) != 0)
    {
      // Выход держится в нуле: интеграл не должен копиться без действия.
      pid_.reset(rpm);
    }
    if (speedTarget_ != 0 && fabsf(output) >= maxOutput_)
    {
      flags_ |= motor_flag::SATURATED;
    }
    event = checkProtections(nowMs);
    break;
  }

  case MotorMode::Coast:
  case MotorMode::Brake:
    break;
  }
  return event;
}

void MotorChannel::setMode(MotorMode mode)
{
  if (mode == MotorMode::Coast || mode == MotorMode::Brake)
  {
    applyStop(mode);
    return;
  }
  const bool entering = mode_ != mode;
  mode_ = mode;
  if (entering)
  {
    slewRemainder_ = 0.0f;
    stallTiming_ = false;
    wrongDirTiming_ = false;
    if (mode == MotorMode::Speed)
    {
      restartPid();
    }
  }
}

void MotorChannel::setPwmTarget(int16_t output)
{
  pwmTarget_ = output;
}

void MotorChannel::setSpeedTarget(int16_t deciRpm)
{
  const bool directionChanged = signOf(deciRpm) != signOf(speedTarget_);
  speedTarget_ = deciRpm;
  if (mode_ == MotorMode::Speed && directionChanged)
  {
    restartPid();
    stallTiming_ = false;
    wrongDirTiming_ = false;
  }
}

void MotorChannel::setMaxOutput(uint16_t maxOutput)
{
  maxOutput_ = maxOutput;
  const float limit = maxOutput_;
  pid_.setOutputLimits(speedTarget_ < 0 ? -limit : 0.0f, speedTarget_ > 0 ? limit : 0.0f);
}

void MotorChannel::setKp(uint16_t centi)
{
  kp_ = centi;
  applyTunings();
}

void MotorChannel::setKi(uint16_t centi)
{
  ki_ = centi;
  applyTunings();
}

void MotorChannel::setKd(uint16_t centi)
{
  kd_ = centi;
  applyTunings();
}

void MotorChannel::setConfig(uint8_t config)
{
  config_ = config & motor_config::MASK;
  driver_.setInvert((config_ & motor_config::INVERT_MOTOR) != 0);
  // Знак счёта мог смениться: начать измерение скорости заново.
  speed_.reset(encoderCount());
  if (mode_ == MotorMode::Speed)
  {
    restartPid();
  }
}

void MotorChannel::setEncoderCount(int32_t count)
{
  const bool invert = (config_ & motor_config::INVERT_ENCODER) != 0;
  encoder_.setCount(invert ? -count : count);
  speed_.rebase(count);
}

void MotorChannel::emergencyBrake()
{
  applyStop(MotorMode::Brake);
}

void MotorChannel::clearFaults()
{
  flags_ &= ~(motor_flag::FAULT_MASK | motor_flag::ENCODER_GLITCH | motor_flag::SATURATED);
  glitchBaseline_ = encoder_.glitches();
  stallTiming_ = false;
  wrongDirTiming_ = false;
}

int16_t MotorChannel::speedDeciRpm() const
{
  const float deci = speed_.rpm() * 10.0f;
  return static_cast<int16_t>(constrain(lroundf(deci), -32768L, 32767L));
}

int32_t MotorChannel::encoderCount() const
{
  const int32_t raw = encoder_.count();
  return (config_ & motor_config::INVERT_ENCODER) != 0 ? -raw : raw;
}

void MotorChannel::applyStop(MotorMode mode)
{
  mode_ = mode;
  if (applied_ != 0)
  {
    zeroSinceMs_ = millis();
  }
  applied_ = 0;
  slewRemainder_ = 0.0f;
  flags_ &= ~(motor_flag::REVERSING | motor_flag::SATURATED);
  if (mode == MotorMode::Brake)
  {
    driver_.brake();
  }
  else
  {
    driver_.coast();
  }
}

void MotorChannel::restartPid()
{
  setMaxOutput(maxOutput_); // пределы по знаку уставки
  pid_.reset(speed_.rpm());
}

void MotorChannel::applyTunings()
{
  pid_.setTunings(kp_ / 100.0f, ki_ / 100.0f, kd_ / 100.0f);
}

void MotorChannel::driveTowards(int16_t target, bool slew, uint32_t nowMs, float dtSeconds)
{
  target = static_cast<int16_t>(constrain(target, -static_cast<int16_t>(maxOutput_),
                                          static_cast<int16_t>(maxOutput_)));
  const int16_t previous = applied_;

  // Никогда не переходить из одного направления в другое напрямую: сначала
  // в ноль, затем выждать REVERSAL_PAUSE_MS.
  int16_t goal = target;
  if (applied_ != 0 && signOf(goal) != 0 && signOf(goal) != signOf(applied_))
  {
    goal = 0;
  }
  else if (applied_ == 0 && goal != 0 && lastSign_ != 0 && signOf(goal) != lastSign_ &&
           nowMs - zeroSinceMs_ < REVERSAL_PAUSE_MS)
  {
    goal = 0;
  }

  if (slew)
  {
    slewRemainder_ += OPEN_LOOP_SLEW_PER_SECOND * dtSeconds;
    const int32_t maxStep = static_cast<int32_t>(slewRemainder_);
    slewRemainder_ -= maxStep;
    const int32_t delta = constrain(goal - applied_, -maxStep, maxStep);
    applied_ = static_cast<int16_t>(applied_ + delta);
  }
  else
  {
    applied_ = goal;
  }

  if (applied_ != 0)
  {
    lastSign_ = signOf(applied_);
  }
  else if (previous != 0)
  {
    zeroSinceMs_ = nowMs;
  }

  flags_ &= ~(motor_flag::REVERSING | motor_flag::SATURATED);
  if (goal != target)
  {
    flags_ |= motor_flag::REVERSING;
  }
  driver_.drive(applied_);
}

MotorEvent MotorChannel::checkProtections(uint32_t nowMs)
{
  const float rpm = speed_.rpm();
  const int16_t magnitude = abs(applied_);

  // При MAX_OUTPUT ниже порога заклинивание ловится на пределе выхода.
  const int16_t stallThreshold = maxOutput_ < STALL_OUTPUT_THRESHOLD
                                     ? static_cast<int16_t>(maxOutput_)
                                     : STALL_OUTPUT_THRESHOLD;
  if (magnitude > 0 && magnitude >= stallThreshold && fabsf(rpm) < STALL_RPM_THRESHOLD)
  {
    if (!stallTiming_)
    {
      stallTiming_ = true;
      stallSinceMs_ = nowMs;
    }
    else if (nowMs - stallSinceMs_ >= STALL_TIMEOUT_MS)
    {
      latchFault(motor_flag::FAULT_STALL);
      return MotorEvent::Stall;
    }
  }
  else
  {
    stallTiming_ = false;
  }

  // Вал вращается против выхода. После реверса это нормально, пока мотор
  // замедляется; ошибка — если за окно он не стал вращаться медленнее.
  const bool wrongDirection = (applied_ > 0 && rpm < -WRONG_DIRECTION_RPM) ||
                              (applied_ < 0 && rpm > WRONG_DIRECTION_RPM);
  if (wrongDirection)
  {
    if (!wrongDirTiming_)
    {
      wrongDirTiming_ = true;
      wrongDirSinceMs_ = nowMs;
      wrongDirRpm_ = fabsf(rpm);
    }
    else if (nowMs - wrongDirSinceMs_ >= WRONG_DIRECTION_TIMEOUT_MS)
    {
      if (fabsf(rpm) >= wrongDirRpm_)
      {
        latchFault(motor_flag::FAULT_DIRECTION);
        return MotorEvent::WrongDirection;
      }
      wrongDirSinceMs_ = nowMs;
      wrongDirRpm_ = fabsf(rpm);
    }
  }
  else
  {
    wrongDirTiming_ = false;
  }
  return MotorEvent::None;
}

void MotorChannel::latchFault(uint8_t flag)
{
  flags_ |= flag;
  applyStop(MotorMode::Brake);
}
