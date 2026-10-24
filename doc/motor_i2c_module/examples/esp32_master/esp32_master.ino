// =============================================================================
// esp32_master.ino - тестовый I2C master для motor_i2c_module
// =============================================================================
//
// Плата: ESP32 DOIT DevKit V1, Arduino-ESP32 2.0.x.
// Подключение к разъёму J1 модуля (обе платы 3.3 В, преобразователь
// уровней не нужен):
//   ESP32 GND          -- J1 1 GND
//   ESP32 VIN (5 В)    -- J1 2 5V   (питание модуля, через диод на модуле)
//   ESP32 GPIO22 (SCL) -- J1 3 SCL  (PB10)
//   ESP32 GPIO21 (SDA) -- J1 4 SDA  (PB11)
// Подтяжки 4.7 кОм к 3.3 В стоят на модуле (R1, R2).
//
// Команды вводятся в Serial Monitor (115200 бод, конец строки LF), список —
// по команде help. Пока включён keepalive (по умолчанию), скетч каждые
// 100 мс читает STATUS — это держит watchdog связи модуля.
//
// Протокол — doc/motor_i2c_module/i2c-protocol.md. Адреса ниже должны
// совпадать с src/motor_i2c_module/register_map.h.

#include <Arduino.h>
#include <Wire.h>

namespace
{
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY = 100000;
constexpr uint8_t MODULE_ADDRESS = 0x20;
constexpr uint16_t KEEPALIVE_PERIOD_MS = 100;

// Общие регистры
constexpr uint8_t REG_WHO_AM_I = 0x00;
constexpr uint8_t REG_STATUS = 0x04;
constexpr uint8_t REG_COMMAND = 0x08;
constexpr uint8_t REG_WATCHDOG_MS = 0x0A;
constexpr uint8_t GLOBAL_BLOCK_SIZE = 0x15; // 0x00..0x14

// Блок мотора n: 0x20 + 0x20 * n
constexpr uint8_t M_MODE = 0x00;
constexpr uint8_t M_CONFIG = 0x01;
constexpr uint8_t M_PWM_TARGET = 0x02;
constexpr uint8_t M_SPEED_TARGET = 0x04;
constexpr uint8_t M_MAX_OUTPUT = 0x06;
constexpr uint8_t M_KP = 0x08;
constexpr uint8_t M_FLAGS = 0x10;
constexpr uint8_t M_ENCODER = 0x18;

constexpr uint8_t CMD_ESTOP = 0x01;
constexpr uint8_t CMD_CLEAR_ERRORS = 0x02;
constexpr uint8_t CMD_RESET_ENCODERS = 0x03;
constexpr uint8_t CMD_COAST_ALL = 0x04;

const char *const MODE_NAMES[] = {"coast", "brake", "pwm", "speed"};

bool keepalive = true;
uint32_t lastKeepaliveMs = 0;
uint16_t telemetryPeriodMs = 0; // 0 — выключена
uint32_t lastTelemetryMs = 0;

uint8_t motorBase(uint8_t motor)
{
  return 0x20 + 0x20 * motor;
}

// CRC-8 SMBus PEC: полином 0x07, начальное значение 0.
uint8_t crc8(uint8_t crc, const uint8_t *data, size_t length)
{
  for (size_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
  }
  return crc;
}

// Запись [регистр][данные][CRC]. Возвращает код endTransmission (0 — ACK).
uint8_t writeRegisters(uint8_t reg, const uint8_t *data, size_t length)
{
  const uint8_t addressByte = MODULE_ADDRESS << 1;
  uint8_t crc = crc8(0, &addressByte, 1);
  crc = crc8(crc, &reg, 1);
  crc = crc8(crc, data, length);

  Wire.beginTransmission(MODULE_ADDRESS);
  Wire.write(reg);
  Wire.write(data, length);
  Wire.write(crc);
  return Wire.endTransmission();
}

// Чтение: [регистр] без CRC, повторный старт, затем length байтов.
bool readRegisters(uint8_t reg, uint8_t *data, size_t length)
{
  Wire.beginTransmission(MODULE_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }
  if (Wire.requestFrom(MODULE_ADDRESS, static_cast<uint8_t>(length)) != length)
  {
    return false;
  }
  for (size_t i = 0; i < length; i++)
  {
    data[i] = Wire.read();
  }
  return true;
}

uint8_t write8(uint8_t reg, uint8_t value)
{
  return writeRegisters(reg, &value, 1);
}

uint8_t write16(uint8_t reg, int32_t value)
{
  const uint8_t bytes[2] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
  return writeRegisters(reg, bytes, 2);
}

uint8_t write32(uint8_t reg, int32_t value)
{
  uint8_t bytes[4];
  for (uint8_t i = 0; i < 4; i++)
  {
    bytes[i] = static_cast<uint8_t>(value >> (8 * i));
  }
  return writeRegisters(reg, bytes, 4);
}

int16_t le16(const uint8_t *p)
{
  return static_cast<int16_t>(p[0] | (p[1] << 8));
}

int32_t le32(const uint8_t *p)
{
  return static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

void report(const char *what, uint8_t result)
{
  if (result == 0)
  {
    Serial.printf("%s: ok\n", what);
  }
  else
  {
    Serial.printf("%s: I2C error %u (2 = NACK адреса, 3 = NACK данных)\n", what, result);
  }
}

void printInfo()
{
  uint8_t g[GLOBAL_BLOCK_SIZE];
  if (!readRegisters(REG_WHO_AM_I, g, sizeof(g)))
  {
    Serial.println("read failed");
    return;
  }
  Serial.printf("who_am_i=0x%02X protocol=%u fw=%u.%u motors=%u\n", g[0x00], g[0x01], g[0x03], g[0x02],
                g[0x07]);
  Serial.printf("status=0x%02X last_error=%u stop_reason=%u stop_motor=%u watchdog=%u ms\n", g[0x04],
                g[0x05], g[0x06], g[0x09], static_cast<uint16_t>(le16(&g[0x0A])));
  Serial.printf("uptime=%lu ms frames_ok=%u rejected=%u reset_flags=0x%02X\n",
                static_cast<unsigned long>(le32(&g[0x0C])), static_cast<uint16_t>(le16(&g[0x10])),
                static_cast<uint16_t>(le16(&g[0x12])), g[0x14]);
}

void printMotor(uint8_t motor)
{
  uint8_t cfg[0x0E];
  uint8_t tel[0x0C];
  if (!readRegisters(motorBase(motor), cfg, sizeof(cfg)) ||
      !readRegisters(motorBase(motor) + M_FLAGS, tel, sizeof(tel)))
  {
    Serial.printf("m%u: read failed\n", motor + 1);
    return;
  }
  const uint8_t mode = cfg[M_MODE];
  Serial.printf("m%u %-5s cfg=0x%02X pwm_tgt=%d speed_tgt=%.1f max=%u kp=%.2f ki=%.2f kd=%.2f | "
                "flags=0x%02X rpm=%.1f out=%d enc=%ld\n",
                motor + 1, mode < 4 ? MODE_NAMES[mode] : "?", cfg[M_CONFIG], le16(&cfg[M_PWM_TARGET]),
                le16(&cfg[M_SPEED_TARGET]) / 10.0f, static_cast<uint16_t>(le16(&cfg[M_MAX_OUTPUT])),
                static_cast<uint16_t>(le16(&cfg[M_KP])) / 100.0f,
                static_cast<uint16_t>(le16(&cfg[M_KP + 2])) / 100.0f,
                static_cast<uint16_t>(le16(&cfg[M_KP + 4])) / 100.0f, tel[0], le16(&tel[2]) / 10.0f,
                le16(&tel[4]), static_cast<long>(le32(&tel[8])));
}

void scanBus()
{
  for (uint8_t address = 0x08; address <= 0x77; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf("found 0x%02X\n", address);
    }
  }
}

void printHelp()
{
  Serial.println(
      "scan                     поиск устройств на шине\n"
      "info                     общие регистры\n"
      "m [1-4]                  состояние одного или всех моторов\n"
      "mode <1-4> <coast|brake|pwm|speed>\n"
      "pwm <1-4> <-1000..1000>  уставка ШИМ, промилле (действует в режиме pwm)\n"
      "speed <1-4> <rpm>        уставка скорости, об/мин (действует в режиме speed)\n"
      "max <1-4> <0..1000>      предел выхода, промилле\n"
      "pid <1-4> <kp> <ki> <kd> коэффициенты PID\n"
      "cfg <1-4> <0..3>         бит0 — инверсия мотора, бит1 — инверсия энкодера\n"
      "enc <1-4> <value>        задать счёт энкодера\n"
      "wd <50..10000>           таймаут watchdog связи, мс\n"
      "estop | clear | coast | zero    команды ESTOP, CLEAR_ERRORS, COAST_ALL, RESET_ENCODERS\n"
      "keepalive <0|1>          периодическое чтение STATUS (держит watchdog)\n"
      "tel <мс|0>               периодический вывод всех моторов\n"
      "raw <reg> <byte>...      запись байтов (hex) с CRC\n"
      "crcbad                   запись с неверным CRC (проверка отказа)");
}

int modeFromName(const String &name)
{
  for (int i = 0; i < 4; i++)
  {
    if (name == MODE_NAMES[i])
    {
      return i;
    }
  }
  return -1;
}

// Номер мотора из строки "1".."4" -> 0..3, или -1.
int motorIndex(const String &token)
{
  const int motor = token.toInt();
  return (motor >= 1 && motor <= 4) ? motor - 1 : -1;
}

void handleLine(String line)
{
  line.trim();
  if (line.isEmpty())
  {
    return;
  }
  String tokens[8];
  int count = 0;
  while (line.length() > 0 && count < 8)
  {
    const int space = line.indexOf(' ');
    tokens[count++] = space < 0 ? line : line.substring(0, space);
    line = space < 0 ? "" : line.substring(space + 1);
    line.trim();
  }
  const String &cmd = tokens[0];
  const int motor = count > 1 ? motorIndex(tokens[1]) : -1;

  if (cmd == "help")
  {
    printHelp();
  }
  else if (cmd == "scan")
  {
    scanBus();
  }
  else if (cmd == "info")
  {
    printInfo();
  }
  else if (cmd == "m")
  {
    if (motor >= 0)
    {
      printMotor(motor);
    }
    else
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        printMotor(i);
      }
    }
  }
  else if (cmd == "mode" && motor >= 0 && count == 3 && modeFromName(tokens[2]) >= 0)
  {
    report("mode", write8(motorBase(motor) + M_MODE, modeFromName(tokens[2])));
  }
  else if (cmd == "pwm" && motor >= 0 && count == 3)
  {
    report("pwm", write16(motorBase(motor) + M_PWM_TARGET, tokens[2].toInt()));
  }
  else if (cmd == "speed" && motor >= 0 && count == 3)
  {
    report("speed", write16(motorBase(motor) + M_SPEED_TARGET, lroundf(tokens[2].toFloat() * 10.0f)));
  }
  else if (cmd == "max" && motor >= 0 && count == 3)
  {
    report("max", write16(motorBase(motor) + M_MAX_OUTPUT, tokens[2].toInt()));
  }
  else if (cmd == "pid" && motor >= 0 && count == 5)
  {
    // KP, KI, KD — три соседних регистра, одна транзакция.
    uint8_t bytes[6];
    for (int i = 0; i < 3; i++)
    {
      const uint16_t centi = static_cast<uint16_t>(lroundf(tokens[2 + i].toFloat() * 100.0f));
      bytes[2 * i] = static_cast<uint8_t>(centi);
      bytes[2 * i + 1] = static_cast<uint8_t>(centi >> 8);
    }
    report("pid", writeRegisters(motorBase(motor) + M_KP, bytes, sizeof(bytes)));
  }
  else if (cmd == "cfg" && motor >= 0 && count == 3)
  {
    report("cfg", write8(motorBase(motor) + M_CONFIG, tokens[2].toInt()));
  }
  else if (cmd == "enc" && motor >= 0 && count == 3)
  {
    report("enc", write32(motorBase(motor) + M_ENCODER, tokens[2].toInt()));
  }
  else if (cmd == "wd" && count == 2)
  {
    report("wd", write16(REG_WATCHDOG_MS, tokens[1].toInt()));
  }
  else if (cmd == "estop")
  {
    report("estop", write8(REG_COMMAND, CMD_ESTOP));
  }
  else if (cmd == "clear")
  {
    report("clear", write8(REG_COMMAND, CMD_CLEAR_ERRORS));
  }
  else if (cmd == "coast")
  {
    report("coast", write8(REG_COMMAND, CMD_COAST_ALL));
  }
  else if (cmd == "zero")
  {
    report("zero", write8(REG_COMMAND, CMD_RESET_ENCODERS));
  }
  else if (cmd == "keepalive" && count == 2)
  {
    keepalive = tokens[1].toInt() != 0;
    Serial.printf("keepalive %s\n", keepalive ? "on" : "off");
  }
  else if (cmd == "tel" && count == 2)
  {
    telemetryPeriodMs = tokens[1].toInt();
  }
  else if (cmd == "raw" && count >= 3)
  {
    uint8_t bytes[6];
    const uint8_t reg = strtoul(tokens[1].c_str(), nullptr, 16);
    for (int i = 2; i < count; i++)
    {
      bytes[i - 2] = strtoul(tokens[i].c_str(), nullptr, 16);
    }
    report("raw", writeRegisters(reg, bytes, count - 2));
  }
  else if (cmd == "crcbad")
  {
    // COMMAND = COAST_ALL с заведомо неверным CRC: модуль должен отклонить.
    Wire.beginTransmission(MODULE_ADDRESS);
    Wire.write(REG_COMMAND);
    Wire.write(CMD_COAST_ALL);
    Wire.write(0x00);
    report("crcbad", Wire.endTransmission());
  }
  else
  {
    Serial.println("? help — список команд");
  }
}
} // namespace

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
  delay(200);
  Serial.println("\nmotor_i2c_module master. help — список команд");
  printInfo();
}

void loop()
{
  static String line;
  while (Serial.available() > 0)
  {
    const char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      handleLine(line);
      line = "";
    }
    else
    {
      line += c;
    }
  }

  const uint32_t now = millis();
  if (keepalive && now - lastKeepaliveMs >= KEEPALIVE_PERIOD_MS)
  {
    lastKeepaliveMs = now;
    uint8_t status = 0;
    readRegisters(REG_STATUS, &status, 1);
  }
  if (telemetryPeriodMs > 0 && now - lastTelemetryMs >= telemetryPeriodMs)
  {
    lastTelemetryMs = now;
    for (uint8_t i = 0; i < 4; i++)
    {
      printMotor(i);
    }
  }
}
