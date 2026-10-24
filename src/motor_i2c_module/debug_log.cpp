#include "debug_log.h"

#include <Arduino.h>
#include <stdarg.h>

namespace
{
uint32_t droppedLines = 0;
}

void debugLog(const char *format, ...)
{
  char line[160];
  va_list args;
  va_start(args, format);
  const int length = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (length <= 0)
  {
    return;
  }
  const size_t size = static_cast<size_t>(length) < sizeof(line) ? length : sizeof(line) - 1;
  if (!Serial || Serial.availableForWrite() < static_cast<int>(size))
  {
    droppedLines++;
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t *>(line), size);
}

uint32_t debugDroppedLines()
{
  return droppedLines;
}
