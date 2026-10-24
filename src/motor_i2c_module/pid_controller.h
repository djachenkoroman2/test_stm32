#pragma once

// =============================================================================
// pid_controller.h - PID-регулятор скорости
// =============================================================================
//
// Не зависит от оборудования. Уставка и измерение — об/мин выходного вала,
// выход — промилле полной скважности ШИМ.
//
// - производная по измерению: выход не «дёргается» при смене уставки;
// - защита от накопления интеграла (условное интегрирование): интеграл
//   перестаёт расти, пока выход упёрся в предел в ту сторону, куда его
//   толкает ошибка;
// - интеграл хранится уже умноженным на ki, поэтому смена ki на ходу не
//   даёт скачка выхода.
//
// Порядок использования: setTunings() и setOutputLimits(), reset() при
// (пере)запуске регулирования, затем update() с постоянным периодом.
class PidController
{
public:
  void setTunings(float kp, float ki, float kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  // Модуль задаёт [0, MAX] для положительной уставки и [-MAX, 0] для
  // отрицательной: регулятор толкает только в сторону уставки, и около нуля
  // не возникает дребезга реверса.
  void setOutputLimits(float minimum, float maximum)
  {
    min_ = minimum;
    max_ = maximum;
    integral_ = clamp(integral_);
  }

  // Безударный запуск: текущее измерение принимается за «предыдущее».
  void reset(float measurement)
  {
    integral_ = 0.0f;
    lastMeasurement_ = measurement;
    output_ = 0.0f;
  }

  float update(float setpoint, float measurement, float dtSeconds)
  {
    if (dtSeconds <= 0.0f)
    {
      return output_;
    }

    const float error = setpoint - measurement;
    const float derivative = -(measurement - lastMeasurement_) / dtSeconds;
    lastMeasurement_ = measurement;

    const float candidateIntegral = integral_ + ki_ * error * dtSeconds;
    const float unclamped = kp_ * error + candidateIntegral + kd_ * derivative;
    const bool saturatedHigh = unclamped > max_ && error > 0.0f;
    const bool saturatedLow = unclamped < min_ && error < 0.0f;
    if (!saturatedHigh && !saturatedLow)
    {
      integral_ = clamp(candidateIntegral);
    }

    output_ = clamp(kp_ * error + integral_ + kd_ * derivative);
    return output_;
  }

  float output() const { return output_; }

private:
  float clamp(float value) const
  {
    if (value > max_)
    {
      return max_;
    }
    if (value < min_)
    {
      return min_;
    }
    return value;
  }

  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float min_ = 0.0f;
  float max_ = 0.0f;
  float integral_ = 0.0f;
  float lastMeasurement_ = 0.0f;
  float output_ = 0.0f;
};
