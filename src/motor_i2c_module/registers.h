#pragma once

// =============================================================================
// registers.h - карта регистров: снимок для чтения и разбор записей
// =============================================================================
//
// Запись (одна транзакция I2C):
//   [регистр] [данные ...] [CRC-8]
// CRC-8 SMBus PEC (полином 0x07, начальное значение 0) считается по байту
// адреса (I2C_ADDRESS << 1), регистру и данным.
//
// Данные могут покрывать несколько соседних регистров, но каждый — целиком,
// и только начинаясь с его первого байта. Запись применяется целиком или
// не применяется вовсе: сначала проверяются все регистры, потом меняется
// состояние.

#include <stdint.h>
#include "i2c_link.h"
#include "module_state.h"
#include "register_map.h"

namespace registers
{
// Заполняет снимок карты регистров текущим состоянием модуля.
void buildImage(const ModuleState &module, uint8_t (&image)[reg::MAP_SIZE]);

// Разбирает и применяет одну запись. Ошибку записывает в module.
void applyFrame(ModuleState &module, const i2c_link::Frame &frame);

// CRC-8 SMBus PEC.
uint8_t crc8(uint8_t crc, const uint8_t *data, uint8_t length);
} // namespace registers
