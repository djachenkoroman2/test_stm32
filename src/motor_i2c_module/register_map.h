#pragma once

// =============================================================================
// register_map.h - карта регистров I2C (протокол версии 1)
// =============================================================================
//
// Полное описание — doc/motor_i2c_module/i2c-protocol.md. Любая правка здесь
// требует правки документации и примера master.
//
// Все многобайтные значения — little-endian. Карта занимает 0x00..0x9F:
//   0x00..0x1F   общие регистры
//   0x20 + 0x20 * n   блок мотора n (n = 0..3)

#include <stdint.h>

namespace reg
{
// --- Общие регистры ---------------------------------------------------------
constexpr uint8_t WHO_AM_I = 0x00;         // u8  RO  DEVICE_ID
constexpr uint8_t PROTOCOL_VERSION = 0x01; // u8  RO
constexpr uint8_t FIRMWARE_VERSION = 0x02; // u16 RO  (major << 8) | minor
constexpr uint8_t STATUS = 0x04;           // u8  RO  биты status::*
constexpr uint8_t LAST_ERROR = 0x05;       // u8  RO  error::*
constexpr uint8_t STOP_REASON = 0x06;      // u8  RO  stop_reason::*
constexpr uint8_t MOTOR_COUNT = 0x07;      // u8  RO
constexpr uint8_t COMMAND = 0x08;          // u8  WO  command::*, читается как 0
constexpr uint8_t STOP_MOTOR = 0x09;       // u8  RO  мотор 0..3 для STALL / WRONG_DIRECTION, иначе 0xFF
constexpr uint8_t WATCHDOG_MS = 0x0A;      // u16 RW  50..10000
constexpr uint8_t UPTIME_MS = 0x0C;        // u32 RO
constexpr uint8_t FRAMES_OK = 0x10;        // u16 RO  принятые записи, с переполнением
constexpr uint8_t FRAMES_REJECTED = 0x12;  // u16 RO  отклонённые записи
constexpr uint8_t RESET_FLAGS = 0x14;      // u8  RO  причина последнего сброса, биты reset_flag::*

// --- Блок мотора: смещения от motorBase(n) -----------------------------------
// 0x00..0x0D — настройка и уставки (RW), можно писать одной транзакцией.
constexpr uint8_t M_MODE = 0x00;         // u8  RW  0 COAST, 1 BRAKE, 2 PWM, 3 SPEED
constexpr uint8_t M_CONFIG = 0x01;       // u8  RW  биты motor_config::*
constexpr uint8_t M_PWM_TARGET = 0x02;   // i16 RW  промилле, -1000..1000
constexpr uint8_t M_SPEED_TARGET = 0x04; // i16 RW  0.1 об/мин, -4000..4000
constexpr uint8_t M_MAX_OUTPUT = 0x06;   // u16 RW  промилле, 0..1000
constexpr uint8_t M_KP = 0x08;           // u16 RW  0.01
constexpr uint8_t M_KI = 0x0A;           // u16 RW  0.01
constexpr uint8_t M_KD = 0x0C;           // u16 RW  0.01
// 0x10..0x1B — телеметрия, можно читать одной транзакцией.
constexpr uint8_t M_FLAGS = 0x10;        // u8  RO  биты motor_flag::*
constexpr uint8_t M_SPEED = 0x12;        // i16 RO  0.1 об/мин
constexpr uint8_t M_OUTPUT = 0x14;       // i16 RO  промилле
constexpr uint8_t M_ENCODER = 0x18;      // i32 RW  отсчёты x4, запись задаёт счёт

constexpr uint8_t MOTOR_BLOCK_BASE = 0x20;
constexpr uint8_t MOTOR_BLOCK_SIZE = 0x20;
constexpr uint8_t MAP_SIZE = MOTOR_BLOCK_BASE + 4 * MOTOR_BLOCK_SIZE; // 0xA0

constexpr uint8_t motorBase(uint8_t motor)
{
  return static_cast<uint8_t>(MOTOR_BLOCK_BASE + motor * MOTOR_BLOCK_SIZE);
}
} // namespace reg

// Биты STATUS. Биты 0..2 блокируют запуск моторов до CLEAR_ERRORS,
// биты 3..7 — защёлки ошибок протокола.
namespace status
{
constexpr uint8_t ESTOP = 1 << 0;        // выполнена аварийная остановка
constexpr uint8_t WATCHDOG = 1 << 1;     // сработал watchdog связи
constexpr uint8_t MOTOR_FAULT = 1 << 2;  // у какого-то мотора защёлкнута ошибка
constexpr uint8_t CRC_ERROR = 1 << 3;
constexpr uint8_t FRAME_ERROR = 1 << 4;  // длина, адрес, запись в RO
constexpr uint8_t VALUE_ERROR = 1 << 5;
constexpr uint8_t STATE_ERROR = 1 << 6;  // запуск мотора при блокировке
constexpr uint8_t RX_OVERFLOW = 1 << 7;  // очередь записей переполнилась
constexpr uint8_t BLOCKING_MASK = ESTOP | WATCHDOG | MOTOR_FAULT;
} // namespace status

// LAST_ERROR: причина последней отклонённой записи.
namespace error
{
constexpr uint8_t NONE = 0;
constexpr uint8_t CRC_MISMATCH = 1; // CRC-8 не совпал
constexpr uint8_t LENGTH = 2;       // нет данных или не хватает байтов до конца регистра
constexpr uint8_t BAD_ADDRESS = 3;  // на адресе не начинается регистр (резерв, середина)
constexpr uint8_t READ_ONLY = 4;    // запись в регистр только для чтения
constexpr uint8_t VALUE = 5;        // значение вне диапазона
constexpr uint8_t STATE = 6;        // запуск мотора при ESTOP / WATCHDOG / ошибке мотора
constexpr uint8_t RX_OVERFLOW = 7;  // запись потеряна: очередь полна
} // namespace error

// STOP_REASON: почему моторы последний раз были остановлены модулем.
namespace stop_reason
{
constexpr uint8_t NONE = 0;
constexpr uint8_t ESTOP = 1;
constexpr uint8_t WATCHDOG = 2;
constexpr uint8_t STALL = 3;            // мотор = STOP_MOTOR
constexpr uint8_t WRONG_DIRECTION = 4;  // мотор = STOP_MOTOR
} // namespace stop_reason

// COMMAND.
namespace command
{
constexpr uint8_t ESTOP = 0x01;          // тормоз всех моторов, защёлка ESTOP
constexpr uint8_t CLEAR_ERRORS = 0x02;   // сброс всех защёлок и STOP_REASON
constexpr uint8_t RESET_ENCODERS = 0x03; // обнулить счёт всех энкодеров
constexpr uint8_t COAST_ALL = 0x04;      // выбег всех моторов, без защёлки
} // namespace command
