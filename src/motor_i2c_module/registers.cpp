#include "registers.h"

#include <Arduino.h>
#include "config.h"

namespace
{
enum Access : uint8_t
{
  RO = 1,
  WO = 2,
  RW = RO | WO,
};

struct RegisterInfo
{
  uint8_t offset;
  uint8_t size;
  bool isSigned;
  Access access;
};

constexpr RegisterInfo GLOBAL_REGISTERS[] = {
    {reg::WHO_AM_I, 1, false, RO},
    {reg::PROTOCOL_VERSION, 1, false, RO},
    {reg::FIRMWARE_VERSION, 2, false, RO},
    {reg::STATUS, 1, false, RO},
    {reg::LAST_ERROR, 1, false, RO},
    {reg::STOP_REASON, 1, false, RO},
    {reg::MOTOR_COUNT, 1, false, RO},
    {reg::COMMAND, 1, false, WO},
    {reg::STOP_MOTOR, 1, false, RO},
    {reg::WATCHDOG_MS, 2, false, RW},
    {reg::UPTIME_MS, 4, false, RO},
    {reg::FRAMES_OK, 2, false, RO},
    {reg::FRAMES_REJECTED, 2, false, RO},
    {reg::RESET_FLAGS, 1, false, RO},
};

constexpr RegisterInfo MOTOR_REGISTERS[] = {
    {reg::M_MODE, 1, false, RW},
    {reg::M_CONFIG, 1, false, RW},
    {reg::M_PWM_TARGET, 2, true, RW},
    {reg::M_SPEED_TARGET, 2, true, RW},
    {reg::M_MAX_OUTPUT, 2, false, RW},
    {reg::M_KP, 2, false, RW},
    {reg::M_KI, 2, false, RW},
    {reg::M_KD, 2, false, RW},
    {reg::M_FLAGS, 1, false, RO},
    {reg::M_SPEED, 2, true, RO},
    {reg::M_OUTPUT, 2, true, RO},
    {reg::M_ENCODER, 4, true, RW},
};

constexpr uint8_t GLOBAL_MOTOR = 0xFF;

// Проверенная запись одного регистра, ждущая применения.
struct PendingWrite
{
  uint8_t motor; // GLOBAL_MOTOR для общих регистров
  uint8_t offset;
  int32_t value;
};

const RegisterInfo *find(const RegisterInfo *table, size_t count, uint8_t offset)
{
  for (size_t i = 0; i < count; i++)
  {
    if (table[i].offset == offset)
    {
      return &table[i];
    }
  }
  return nullptr;
}

// Находит регистр, начинающийся ровно на address.
const RegisterInfo *lookup(uint8_t address, uint8_t &motor, uint8_t &offset)
{
  if (address >= reg::MAP_SIZE)
  {
    return nullptr;
  }
  if (address < reg::MOTOR_BLOCK_BASE)
  {
    motor = GLOBAL_MOTOR;
    offset = address;
    return find(GLOBAL_REGISTERS, sizeof(GLOBAL_REGISTERS) / sizeof(GLOBAL_REGISTERS[0]), offset);
  }
  motor = (address - reg::MOTOR_BLOCK_BASE) / reg::MOTOR_BLOCK_SIZE;
  offset = (address - reg::MOTOR_BLOCK_BASE) % reg::MOTOR_BLOCK_SIZE;
  return find(MOTOR_REGISTERS, sizeof(MOTOR_REGISTERS) / sizeof(MOTOR_REGISTERS[0]), offset);
}

int32_t decode(const uint8_t *bytes, const RegisterInfo &info)
{
  uint32_t raw = 0;
  for (uint8_t i = 0; i < info.size; i++)
  {
    raw |= static_cast<uint32_t>(bytes[i]) << (8 * i);
  }
  if (info.isSigned && info.size < 4)
  {
    const uint32_t signBit = 1UL << (8 * info.size - 1);
    if (raw & signBit)
    {
      raw |= ~((signBit << 1) - 1);
    }
  }
  return static_cast<int32_t>(raw);
}

bool inRange(int32_t value, int32_t minimum, int32_t maximum)
{
  return value >= minimum && value <= maximum;
}

uint8_t validateGlobal(uint8_t offset, int32_t value)
{
  switch (offset)
  {
  case reg::COMMAND:
    return inRange(value, command::ESTOP, command::COAST_ALL) ? error::NONE : error::VALUE;
  case reg::WATCHDOG_MS:
    return inRange(value, COMM_WATCHDOG_MIN_MS, COMM_WATCHDOG_MAX_MS) ? error::NONE : error::VALUE;
  default:
    return error::READ_ONLY;
  }
}

uint8_t validateMotor(const ModuleState &module, uint8_t motor, uint8_t offset, int32_t value)
{
  constexpr int32_t maxDeciRpm = static_cast<int32_t>(MAX_TARGET_RPM * 10.0f);
  switch (offset)
  {
  case reg::M_MODE:
    if (!inRange(value, static_cast<int32_t>(MotorMode::Coast), static_cast<int32_t>(MotorMode::Speed)))
    {
      return error::VALUE;
    }
    if ((value == static_cast<int32_t>(MotorMode::Pwm) || value == static_cast<int32_t>(MotorMode::Speed)) &&
        ((module.latches & status::BLOCKING_MASK) != 0 || module.motors[motor].faulted()))
    {
      return error::STATE;
    }
    return error::NONE;
  case reg::M_CONFIG:
    return (value & ~motor_config::MASK) == 0 ? error::NONE : error::VALUE;
  case reg::M_PWM_TARGET:
    return inRange(value, -OUTPUT_FULL_SCALE, OUTPUT_FULL_SCALE) ? error::NONE : error::VALUE;
  case reg::M_SPEED_TARGET:
    return inRange(value, -maxDeciRpm, maxDeciRpm) ? error::NONE : error::VALUE;
  case reg::M_MAX_OUTPUT:
    return inRange(value, 0, OUTPUT_FULL_SCALE) ? error::NONE : error::VALUE;
  case reg::M_KP:
  case reg::M_KI:
  case reg::M_KD:
  case reg::M_ENCODER:
    return error::NONE; // весь диапазон типа допустим
  default:
    return error::READ_ONLY;
  }
}

void applyGlobal(ModuleState &module, uint8_t offset, int32_t value)
{
  switch (offset)
  {
  case reg::COMMAND:
    switch (value)
    {
    case command::ESTOP:
      module.stopAll(stop_reason::ESTOP);
      break;
    case command::CLEAR_ERRORS:
      module.clearErrors();
      break;
    case command::RESET_ENCODERS:
      for (MotorChannel &motor : module.motors)
      {
        motor.setEncoderCount(0);
      }
      break;
    case command::COAST_ALL:
      for (MotorChannel &motor : module.motors)
      {
        motor.setMode(MotorMode::Coast);
      }
      break;
    }
    break;
  case reg::WATCHDOG_MS:
    module.watchdogMs = static_cast<uint16_t>(value);
    break;
  }
}

void applyMotor(MotorChannel &motor, uint8_t offset, int32_t value)
{
  switch (offset)
  {
  case reg::M_MODE:
    motor.setMode(static_cast<MotorMode>(value));
    break;
  case reg::M_CONFIG:
    motor.setConfig(static_cast<uint8_t>(value));
    break;
  case reg::M_PWM_TARGET:
    motor.setPwmTarget(static_cast<int16_t>(value));
    break;
  case reg::M_SPEED_TARGET:
    motor.setSpeedTarget(static_cast<int16_t>(value));
    break;
  case reg::M_MAX_OUTPUT:
    motor.setMaxOutput(static_cast<uint16_t>(value));
    break;
  case reg::M_KP:
    motor.setKp(static_cast<uint16_t>(value));
    break;
  case reg::M_KI:
    motor.setKi(static_cast<uint16_t>(value));
    break;
  case reg::M_KD:
    motor.setKd(static_cast<uint16_t>(value));
    break;
  case reg::M_ENCODER:
    motor.setEncoderCount(value);
    break;
  }
}

// Проверяет запись целиком. Возвращает код ошибки; при успехе writes/count
// содержат регистры для применения.
uint8_t parse(const ModuleState &module, const i2c_link::Frame &frame,
              PendingWrite (&writes)[i2c_link::MAX_FRAME], uint8_t &count)
{
  count = 0;
  uint8_t dataLength;
  if (I2C_WRITE_CRC_REQUIRED)
  {
    if (frame.length < 3)
    {
      return error::LENGTH;
    }
    const uint8_t addressByte = static_cast<uint8_t>(I2C_ADDRESS << 1);
    uint8_t crc = registers::crc8(0, &addressByte, 1);
    crc = registers::crc8(crc, frame.bytes, frame.length - 1);
    if (crc != frame.bytes[frame.length - 1])
    {
      return error::CRC_MISMATCH;
    }
    dataLength = frame.length - 2;
  }
  else
  {
    dataLength = frame.length - 1;
  }

  const uint8_t *data = frame.bytes + 1;
  uint16_t address = frame.bytes[0];
  uint8_t position = 0;
  while (position < dataLength)
  {
    uint8_t motor = 0;
    uint8_t offset = 0;
    const RegisterInfo *info = address < 0x100 ? lookup(static_cast<uint8_t>(address), motor, offset) : nullptr;
    if (info == nullptr)
    {
      return error::BAD_ADDRESS;
    }
    if ((info->access & WO) == 0)
    {
      return error::READ_ONLY;
    }
    if (position + info->size > dataLength)
    {
      return error::LENGTH;
    }
    const int32_t value = decode(data + position, *info);
    const uint8_t code = motor == GLOBAL_MOTOR ? validateGlobal(offset, value)
                                               : validateMotor(module, motor, offset, value);
    if (code != error::NONE)
    {
      return code;
    }
    writes[count++] = {motor, offset, value};
    position += info->size;
    address += info->size;
  }
  return error::NONE;
}

void put8(uint8_t (&image)[reg::MAP_SIZE], uint8_t address, uint8_t value)
{
  image[address] = value;
}

void put16(uint8_t (&image)[reg::MAP_SIZE], uint8_t address, uint16_t value)
{
  image[address] = static_cast<uint8_t>(value);
  image[address + 1] = static_cast<uint8_t>(value >> 8);
}

void put32(uint8_t (&image)[reg::MAP_SIZE], uint8_t address, uint32_t value)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    image[address + i] = static_cast<uint8_t>(value >> (8 * i));
  }
}
} // namespace

namespace registers
{
void buildImage(const ModuleState &module, uint8_t (&image)[reg::MAP_SIZE])
{
  memset(image, 0, sizeof(image));
  put8(image, reg::WHO_AM_I, DEVICE_ID);
  put8(image, reg::PROTOCOL_VERSION, PROTOCOL_VERSION);
  put16(image, reg::FIRMWARE_VERSION, (FIRMWARE_VERSION_MAJOR << 8) | FIRMWARE_VERSION_MINOR);
  put8(image, reg::STATUS, module.status());
  put8(image, reg::LAST_ERROR, module.lastError);
  put8(image, reg::STOP_REASON, module.stopReason);
  put8(image, reg::MOTOR_COUNT, MOTOR_COUNT);
  put8(image, reg::STOP_MOTOR, module.stopMotor);
  put16(image, reg::WATCHDOG_MS, module.watchdogMs);
  put32(image, reg::UPTIME_MS, millis());
  put16(image, reg::FRAMES_OK, module.framesOk);
  put16(image, reg::FRAMES_REJECTED, module.framesRejected);
  put8(image, reg::RESET_FLAGS, module.resetFlags);

  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
  {
    const MotorChannel &motor = module.motors[i];
    const uint8_t base = reg::motorBase(i);
    put8(image, base + reg::M_MODE, static_cast<uint8_t>(motor.mode()));
    put8(image, base + reg::M_CONFIG, motor.config());
    put16(image, base + reg::M_PWM_TARGET, static_cast<uint16_t>(motor.pwmTarget()));
    put16(image, base + reg::M_SPEED_TARGET, static_cast<uint16_t>(motor.speedTarget()));
    put16(image, base + reg::M_MAX_OUTPUT, motor.maxOutput());
    put16(image, base + reg::M_KP, motor.kp());
    put16(image, base + reg::M_KI, motor.ki());
    put16(image, base + reg::M_KD, motor.kd());
    put8(image, base + reg::M_FLAGS, motor.flags());
    put16(image, base + reg::M_SPEED, static_cast<uint16_t>(motor.speedDeciRpm()));
    put16(image, base + reg::M_OUTPUT, static_cast<uint16_t>(motor.output()));
    put32(image, base + reg::M_ENCODER, static_cast<uint32_t>(motor.encoderCount()));
  }
}

void applyFrame(ModuleState &module, const i2c_link::Frame &frame)
{
  PendingWrite writes[i2c_link::MAX_FRAME];
  uint8_t count = 0;
  const uint8_t code = parse(module, frame, writes, count);
  if (code != error::NONE)
  {
    module.rejectFrame(code);
    return;
  }
  for (uint8_t i = 0; i < count; i++)
  {
    const PendingWrite &write = writes[i];
    if (write.motor == GLOBAL_MOTOR)
    {
      applyGlobal(module, write.offset, write.value);
    }
    else
    {
      applyMotor(module.motors[write.motor], write.offset, write.value);
    }
  }
  module.framesOk++;
}

uint8_t crc8(uint8_t crc, const uint8_t *data, uint8_t length)
{
  for (uint8_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
} // namespace registers
