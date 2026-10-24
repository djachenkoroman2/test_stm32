#pragma once

// =============================================================================
// speed_meter.h - скорость выходного вала по приращению счёта энкодера
// =============================================================================

#include <stdint.h>
#include "config.h"

class SpeedMeter
{
public:
  void reset(int32_t count)
  {
    lastCount_ = count;
    rpm_ = 0.0f;
  }

  // Вызывается раз в период контура с накопленным счётом энкодера.
  void update(int32_t count, float dtSeconds)
  {
    const int32_t delta = count - lastCount_;
    lastCount_ = count;
    if (dtSeconds <= 0.0f)
    {
      return;
    }
    const float raw = static_cast<float>(delta) * 60.0f / (COUNTS_PER_OUTPUT_REV * dtSeconds);
    rpm_ += SPEED_FILTER_ALPHA * (raw - rpm_);
  }

  // Счёт изменён извне (запись регистра ENCODER): приращение не считается.
  void rebase(int32_t count) { lastCount_ = count; }

  float rpm() const { return rpm_; }

private:
  int32_t lastCount_ = 0;
  float rpm_ = 0.0f;
};
