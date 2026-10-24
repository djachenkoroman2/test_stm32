#pragma once

// =============================================================================
// config.h - все аппаратные и настроечные параметры модуля
// =============================================================================
//
// Оборудование: Blue Pill (STM32F103C8T6) + два двухканальных драйвера ZK-5AD
// (TA6586) + четыре мотор-редуктора JGB37-520 с энкодерами Холла. Модуль —
// I2C slave: master задаёт режимы и уставки, модуль держит скорость по PID.
//
// Единицы во всём проекте:
//   - выход на мотор: промилле полной скважности ШИМ, -1000..1000, знак —
//     направление (+ вперёд, - назад);
//   - скорость: об/мин ВЫХОДНОГО вала редуктора; по I2C — в 0.1 об/мин;
//   - позиция энкодера: отсчёты после квадратурного декодирования x4;
//   - время: миллисекунды (суффикс _MS), если не сказано иное.
//
// Распределение таймеров (подробно — doc/motor_i2c_module/wiring.md):
//   TIM2 — ШИМ моторов 1 и 2, TIM3 — ШИМ моторов 3 и 4,
//   TIM1, TIM4 — аппаратные энкодеры моторов 1 и 2,
//   EXTI 12..15 — программные энкодеры моторов 3 и 4.

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Версии
// -----------------------------------------------------------------------------

constexpr uint8_t DEVICE_ID = 0x4D;        // регистр WHO_AM_I, 'M'
constexpr uint8_t PROTOCOL_VERSION = 1;    // меняется при несовместимых правках карты регистров
constexpr uint8_t FIRMWARE_VERSION_MAJOR = 1;
constexpr uint8_t FIRMWARE_VERSION_MINOR = 0;

// -----------------------------------------------------------------------------
// I2C
// -----------------------------------------------------------------------------

// I2C2: PB10/PB11 не пересекаются ни с одним таймером, занятым модулем.
// I2C1 (PB6/PB7) делит пины с TIM4, который считает энкодер мотора 2.
constexpr uint8_t I2C_ADDRESS = 0x20;      // 7-битный адрес
constexpr uint32_t I2C_SCL_PIN = PB10;
constexpr uint32_t I2C_SDA_PIN = PB11;
// Каждая запись данных заканчивается байтом CRC-8 (SMBus PEC). Без него
// повреждённая уставка дошла бы до моторов.
constexpr bool I2C_WRITE_CRC_REQUIRED = true;

// Watchdog связи: если ни один мотор не в активном режиме (PWM / SPEED),
// он не работает. Иначе при молчании master дольше таймаута — тормоз всех
// моторов. Таймаут меняется регистром WATCHDOG_MS.
constexpr uint16_t COMM_WATCHDOG_DEFAULT_MS = 500;
constexpr uint16_t COMM_WATCHDOG_MIN_MS = 50;
constexpr uint16_t COMM_WATCHDOG_MAX_MS = 10000;

// Аппаратный IWDG: перезапуск MCU, если loop() завис. После сброса входы
// ZK-5AD плавают, поэтому на них рекомендуются подтяжки к GND (wiring.md).
constexpr uint32_t IWDG_TIMEOUT_US = 250000;

// -----------------------------------------------------------------------------
// Пины моторов
// -----------------------------------------------------------------------------

constexpr uint8_t MOTOR_COUNT = 4;

enum class EncoderKind : uint8_t
{
  Timer, // аппаратный режим энкодера таймера (TI1/TI2 = CH1/CH2)
  Exti,  // внешние прерывания на оба фронта A и B, декодирование x4
};

struct MotorPins
{
  // ZK-5AD: ШИМ подаётся на один вход канала, на другой — LOW.
  TIM_TypeDef *pwmTimer;
  uint8_t in1Channel; // канал таймера для IN1 / IN3
  PinName in1Pin;
  uint8_t in2Channel; // канал таймера для IN2 / IN4
  PinName in2Pin;
  // Энкодер JGB37-520: фаза A (C1), фаза B (C2).
  EncoderKind encoderKind;
  TIM_TypeDef *encoderTimer; // только для EncoderKind::Timer
  uint32_t encoderAPin;
  uint32_t encoderBPin;
};

// PA7, PB0, PB1 по умолчанию сопоставлены комплементарным каналам TIM1
// (TIM1_CHxN), поэтому для TIM3 нужны альтернативные имена *_ALT1.
static const MotorPins MOTOR_PINS[MOTOR_COUNT] = {
    // Мотор 1: ZK-5AD №1, канал A (IN1/IN2)
    {TIM2, 1, PA_0, 2, PA_1, EncoderKind::Timer, TIM1, PA8, PA9},
    // Мотор 2: ZK-5AD №1, канал B (IN3/IN4)
    {TIM2, 3, PA_2, 4, PA_3, EncoderKind::Timer, TIM4, PB6, PB7},
    // Мотор 3: ZK-5AD №2, канал A (IN1/IN2)
    {TIM3, 1, PA_6, 2, PA_7_ALT1, EncoderKind::Exti, nullptr, PB12, PB13},
    // Мотор 4: ZK-5AD №2, канал B (IN3/IN4)
    {TIM3, 3, PB_0_ALT1, 4, PB_1_ALT1, EncoderKind::Exti, nullptr, PB14, PB15},
};

// Светодиод Blue Pill, активен низким уровнем.
constexpr uint32_t LED_PIN = PC13;

// -----------------------------------------------------------------------------
// ШИМ и выход на мотор
// -----------------------------------------------------------------------------

// 72 МГц / 20 кГц = 3600 тактов на период: выше слышимого диапазона,
// разрешение лучше 0.03 %.
constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr int16_t OUTPUT_FULL_SCALE = 1000;    // промилле
constexpr uint16_t DEFAULT_MAX_OUTPUT = 900;   // предел выхода по умолчанию, 90 %
// Темп разгона в режиме PWM (без обратной связи), промилле в секунду.
constexpr uint16_t OPEN_LOOP_SLEW_PER_SECOND = 1500;
// Минимальное время на нулевом выходе перед вращением в обратную сторону.
constexpr uint16_t REVERSAL_PAUSE_MS = 300;

// -----------------------------------------------------------------------------
// Мотор и энкодер
// -----------------------------------------------------------------------------

// JGB37-520 (verify — сверьте с вашей версией мотора).
constexpr float GEAR_RATIO = 30.0f;                    // 1:30 -> ~330 об/мин при 12 В
constexpr uint16_t ENCODER_PULSES_PER_MOTOR_REV = 11;  // на одну фазу
constexpr uint8_t ENCODER_DECODING = 4;                // оба фронта A и B
constexpr float COUNTS_PER_OUTPUT_REV =
    ENCODER_PULSES_PER_MOTOR_REV * ENCODER_DECODING * GEAR_RATIO; // 1320

// Внутренние подтяжки входов энкодера к 3.3 В. Выключите, если энкодер
// питается от 5 В: подтяжка к VDD на пине с 5 В нежелательна.
constexpr bool ENCODER_INTERNAL_PULLUP = true;
// Цифровой фильтр входов аппаратных энкодеров (ICxF): 0b1001 = fDTS/8, N=8,
// ~0.9 мкс. Фронты на номинальной скорости идут не чаще ~70 мкс.
constexpr uint8_t ENCODER_TIMER_FILTER = 0b1001;

// Инверсия по умолчанию (биты регистра CONFIG, меняются по I2C).
constexpr bool DEFAULT_INVERT_MOTOR = false;
constexpr bool DEFAULT_INVERT_ENCODER = false;

// -----------------------------------------------------------------------------
// Контур регулирования
// -----------------------------------------------------------------------------

constexpr uint16_t CONTROL_PERIOD_MS = 10;
// EMA скорости: rpm += alpha * (raw - rpm). Один отсчёт за 10 мс — это
// ~4.5 об/мин, без фильтра измерение слишком грубое.
constexpr float SPEED_FILTER_ALPHA = 0.3f;
constexpr float MAX_TARGET_RPM = 400.0f;

// PID: выход в промилле, ошибка в об/мин. По I2C коэффициенты передаются
// в сотых (u16), поэтому диапазон 0..655.35.
constexpr float DEFAULT_KP = 1.5f;  // промилле на 1 об/мин
constexpr float DEFAULT_KI = 8.0f;  // промилле на (об/мин * с)
constexpr float DEFAULT_KD = 0.0f;  // промилле на (об/мин / с)

// Защиты режима SPEED.
// Заклинивание: |выход| >= порога, а вал медленнее STALL_RPM_THRESHOLD
// дольше STALL_TIMEOUT_MS -> тормоз. Ловит заклинивший вал и отключённый энкодер.
constexpr int16_t STALL_OUTPUT_THRESHOLD = 500;
constexpr float STALL_RPM_THRESHOLD = 3.0f;
constexpr uint16_t STALL_TIMEOUT_MS = 1000;
// Не то направление: вал вращается против выхода быстрее
// WRONG_DIRECTION_RPM и не замедляется за WRONG_DIRECTION_TIMEOUT_MS ->
// тормоз. Ловит перепутанную полярность энкодера или мотора.
constexpr float WRONG_DIRECTION_RPM = 20.0f;
constexpr uint16_t WRONG_DIRECTION_TIMEOUT_MS = 300;

// -----------------------------------------------------------------------------
// Отладка
// -----------------------------------------------------------------------------

// Строка состояния в Serial (USB CDC). Печать неблокирующая: если буфер
// USB занят, строка пропускается, контур не ждёт.
constexpr uint16_t DEBUG_PERIOD_MS = 500;

// -----------------------------------------------------------------------------
// Проверки при компиляции
// -----------------------------------------------------------------------------

static_assert(DEFAULT_MAX_OUTPUT > 0 && DEFAULT_MAX_OUTPUT <= OUTPUT_FULL_SCALE,
              "DEFAULT_MAX_OUTPUT must be within 1..OUTPUT_FULL_SCALE");
static_assert(COMM_WATCHDOG_DEFAULT_MS >= COMM_WATCHDOG_MIN_MS &&
                  COMM_WATCHDOG_DEFAULT_MS <= COMM_WATCHDOG_MAX_MS,
              "COMM_WATCHDOG_DEFAULT_MS out of range");
static_assert(CONTROL_PERIOD_MS > 0, "Control period must be positive");
static_assert(MAX_TARGET_RPM * 10.0f <= 32767.0f, "Speed must fit int16 in 0.1 rpm");
static_assert(DEFAULT_KP * 100.0f <= 65535.0f && DEFAULT_KI * 100.0f <= 65535.0f &&
                  DEFAULT_KD * 100.0f <= 65535.0f,
              "PID gains must fit u16 in 0.01 units");
static_assert(I2C_ADDRESS >= 0x08 && I2C_ADDRESS <= 0x77, "Reserved I2C address");
