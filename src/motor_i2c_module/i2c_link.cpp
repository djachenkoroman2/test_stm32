#include "i2c_link.h"

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace
{
TwoWire bus(I2C_SDA_PIN, I2C_SCL_PIN);

constexpr uint8_t QUEUE_SIZE = 8; // степень двойки

i2c_link::Frame queue[QUEUE_SIZE];
volatile uint8_t queueHead = 0; // пишет прерывание
volatile uint8_t queueTail = 0; // пишет loop()
volatile uint16_t droppedFrames = 0;

uint8_t isrImage[reg::MAP_SIZE];
volatile uint8_t readPointer = 0;
volatile uint32_t lastContact = 0;

void onReceive(int count)
{
  lastContact = millis();
  uint8_t buffer[i2c_link::MAX_FRAME];
  uint8_t length = 0;
  // Всё нужно вычитать: Wire не примет следующую запись, пока буфер не пуст.
  while (bus.available() > 0)
  {
    const int value = bus.read();
    if (length < sizeof(buffer))
    {
      buffer[length++] = static_cast<uint8_t>(value);
    }
  }
  (void)count;
  if (length == 0)
  {
    return;
  }
  readPointer = buffer[0];
  if (length == 1)
  {
    return; // только указатель для следующего чтения
  }

  const uint8_t head = queueHead;
  const uint8_t next = (head + 1) & (QUEUE_SIZE - 1);
  if (next == queueTail)
  {
    droppedFrames = droppedFrames + 1;
    return;
  }
  queue[head].length = length;
  memcpy(queue[head].bytes, buffer, length);
  queueHead = next;
}

void onRequest()
{
  lastContact = millis();
  // Master прочитает сколько захочет; отдаём не больше буфера Wire.
  uint8_t buffer[i2c_link::MAX_FRAME];
  const uint8_t start = readPointer;
  for (uint8_t i = 0; i < sizeof(buffer); i++)
  {
    const uint16_t address = start + i;
    buffer[i] = address < reg::MAP_SIZE ? isrImage[address] : 0;
  }
  bus.write(buffer, sizeof(buffer));
}
} // namespace

namespace i2c_link
{
void begin()
{
  bus.begin(I2C_ADDRESS);
  bus.onReceive(onReceive);
  bus.onRequest(onRequest);
}

bool popFrame(Frame &frame)
{
  const uint8_t tail = queueTail;
  if (tail == queueHead)
  {
    return false;
  }
  frame = queue[tail];
  queueTail = (tail + 1) & (QUEUE_SIZE - 1);
  return true;
}

void publish(const uint8_t (&image)[reg::MAP_SIZE])
{
  // 160 байт копируются за ~2 мкс; прерывание I2C в это время ждёт
  // (растяжка SCL), зато master никогда не читает наполовину старый снимок.
  noInterrupts();
  memcpy(isrImage, image, sizeof(isrImage));
  interrupts();
}

uint32_t lastContactMs()
{
  return lastContact;
}

uint16_t takeDroppedFrames()
{
  noInterrupts();
  const uint16_t dropped = droppedFrames;
  droppedFrames = 0;
  interrupts();
  return dropped;
}
} // namespace i2c_link
