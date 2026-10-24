#include "quadrature_encoder.h"

namespace
{
// Состояние программного энкодера, общее с обработчиком прерывания.
struct ExtiState
{
  GPIO_TypeDef *portA;
  GPIO_TypeDef *portB;
  uint32_t maskA;
  uint32_t maskB;
  volatile int32_t count;
  volatile uint32_t glitches;
  uint8_t last; // (A << 1) | B
};

ExtiState extiStates[MOTOR_COUNT] = {};

// Индекс: (прежнее состояние << 2) | новое, состояние = (A << 1) | B.
// A опережает B: 00 -> 10 -> 11 -> 01 -> 00, каждый шаг +1.
// 2 — недопустимый переход (сменились оба сигнала).
constexpr int8_t TRANSITION[16] = {
    0, -1, 1, 2,
    1, 0, 2, -1,
    -1, 2, 0, 1,
    2, 1, -1, 0,
};

inline uint8_t readState(const ExtiState &s)
{
  const uint8_t a = (s.portA->IDR & s.maskA) ? 1 : 0;
  const uint8_t b = (s.portB->IDR & s.maskB) ? 1 : 0;
  return static_cast<uint8_t>((a << 1) | b);
}

template <uint8_t Slot>
void onEdge()
{
  ExtiState &s = extiStates[Slot];
  const uint8_t now = readState(s);
  const int8_t step = TRANSITION[(s.last << 2) | now];
  if (step == 2)
  {
    s.glitches = s.glitches + 1;
  }
  else
  {
    s.count = s.count + step;
  }
  s.last = now;
}

using EdgeHandler = void (*)();
constexpr EdgeHandler EDGE_HANDLERS[MOTOR_COUNT] = {onEdge<0>, onEdge<1>, onEdge<2>, onEdge<3>};

void enableTimerClock(TIM_TypeDef *timer)
{
  if (timer == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (timer == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if (timer == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if (timer == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}
} // namespace

void QuadratureEncoder::begin(const MotorPins &pins, uint8_t slot)
{
  kind_ = pins.encoderKind;
  slot_ = slot;
  // На F1 вход таймера — обычный режим входа GPIO, альтернативная функция
  // для входов не настраивается.
  const uint32_t mode = ENCODER_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(pins.encoderAPin, mode);
  pinMode(pins.encoderBPin, mode);

  if (kind_ == EncoderKind::Timer)
  {
    timer_ = pins.encoderTimer;
    enableTimerClock(timer_);
    timer_->CR1 = 0;
    timer_->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // режим энкодера 3: оба входа
    timer_->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                    (ENCODER_TIMER_FILTER << TIM_CCMR1_IC1F_Pos) |
                    (ENCODER_TIMER_FILTER << TIM_CCMR1_IC2F_Pos);
    timer_->CCER = 0; // без инверсии входов
    timer_->PSC = 0;
    timer_->ARR = 0xFFFF;
    timer_->CNT = 0;
    timer_->EGR = TIM_EGR_UG;
    timer_->CR1 = TIM_CR1_CEN;
    lastTimerCount_ = 0;
    total_ = 0;
    return;
  }

  ExtiState &s = extiStates[slot_];
  s.portA = digitalPinToPort(pins.encoderAPin);
  s.portB = digitalPinToPort(pins.encoderBPin);
  s.maskA = digitalPinToBitMask(pins.encoderAPin);
  s.maskB = digitalPinToBitMask(pins.encoderBPin);
  s.count = 0;
  s.glitches = 0;
  s.last = readState(s);
  attachInterrupt(digitalPinToInterrupt(pins.encoderAPin), EDGE_HANDLERS[slot_], CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins.encoderBPin), EDGE_HANDLERS[slot_], CHANGE);
}

void QuadratureEncoder::update()
{
  if (kind_ != EncoderKind::Timer || timer_ == nullptr)
  {
    return;
  }
  const uint16_t now = static_cast<uint16_t>(timer_->CNT);
  total_ += static_cast<int16_t>(now - lastTimerCount_);
  lastTimerCount_ = now;
}

int32_t QuadratureEncoder::count() const
{
  if (kind_ == EncoderKind::Timer)
  {
    return total_;
  }
  return extiStates[slot_].count; // 32-битное чтение на Cortex-M3 атомарно
}

void QuadratureEncoder::setCount(int32_t value)
{
  if (kind_ == EncoderKind::Timer)
  {
    update();
    total_ = value;
    return;
  }
  noInterrupts();
  extiStates[slot_].count = value;
  interrupts();
}

uint32_t QuadratureEncoder::glitches() const
{
  return kind_ == EncoderKind::Exti ? extiStates[slot_].glitches : 0;
}
