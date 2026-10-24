#include "zk5ad_driver.h"

namespace
{
// Один объект HardwareTimer на таймер: TIM2 и TIM3 делят по два канала ZK-5AD.
struct PwmTimerSlot
{
  TIM_TypeDef *instance;
  HardwareTimer *timer;
};

PwmTimerSlot pwmTimers[2] = {};

HardwareTimer *pwmTimerFor(TIM_TypeDef *instance)
{
  for (PwmTimerSlot &slot : pwmTimers)
  {
    if (slot.instance == instance)
    {
      return slot.timer;
    }
    if (slot.instance == nullptr)
    {
      slot.instance = instance;
      slot.timer = new HardwareTimer(instance);
      slot.timer->setOverflow(PWM_FREQUENCY_HZ, HERTZ_FORMAT);
      return slot.timer;
    }
  }
  return nullptr; // в config.h больше двух таймеров ШИМ
}
} // namespace

void Zk5adChannel::begin(const MotorPins &pins, bool invert)
{
  const uint32_t in1 = pinNametoDigitalPin(pins.in1Pin);
  const uint32_t in2 = pinNametoDigitalPin(pins.in2Pin);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  invert_ = invert;
  in1Channel_ = pins.in1Channel;
  in2Channel_ = pins.in2Channel;
  timer_ = pwmTimerFor(pins.pwmTimer);
  if (timer_ == nullptr)
  {
    return;
  }
  periodTicks_ = timer_->getOverflow(TICK_FORMAT);
  timer_->setMode(in1Channel_, TIMER_OUTPUT_COMPARE_PWM1, pins.in1Pin);
  timer_->setMode(in2Channel_, TIMER_OUTPUT_COMPARE_PWM1, pins.in2Pin);
  write(0, 0);
}

void Zk5adChannel::startTimers()
{
  for (PwmTimerSlot &slot : pwmTimers)
  {
    if (slot.timer != nullptr)
    {
      slot.timer->resume();
    }
  }
}

void Zk5adChannel::drive(int16_t output)
{
  if (invert_)
  {
    output = -output;
  }
  const int32_t magnitude = constrain(abs(output), 0, OUTPUT_FULL_SCALE);
  const uint32_t ticks = static_cast<uint32_t>(magnitude) * periodTicks_ / OUTPUT_FULL_SCALE;
  if (output > 0)
  {
    write(ticks, 0); // ШИМ на IN1 -> вперёд
  }
  else if (output < 0)
  {
    write(0, ticks); // ШИМ на IN2 -> назад
  }
  else
  {
    write(0, 0);
  }
}

void Zk5adChannel::coast()
{
  write(0, 0);
}

void Zk5adChannel::brake()
{
  write(periodTicks_, periodTicks_);
}

void Zk5adChannel::write(uint32_t in1Ticks, uint32_t in2Ticks)
{
  if (timer_ == nullptr)
  {
    return;
  }
  // CCR с preload: новое значение применяется с начала следующего периода.
  timer_->setCaptureCompare(in1Channel_, in1Ticks, TICK_COMPARE_FORMAT);
  timer_->setCaptureCompare(in2Channel_, in2Ticks, TICK_COMPARE_FORMAT);
}
