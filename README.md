# test_stm32

PlatformIO-проект для экспериментов с платой Blue Pill (STM32F103C8T6) на Arduino-ядре stm32duino.

## Окружения

| Окружение | Код | Назначение |
|---|---|---|
| `genericSTM32F103C8` | `src/genericSTM32F103C8/` | Базовый шаблон |
| `bluepill_blink` | `src/bluepill_blink/` | Тест платы миганием светодиода, прошивка через ST-Link |
| `bluepill_blink_hid` | `src/bluepill_blink/` | Тот же тест, прошивка по USB через HID Bootloader, Serial через USB CDC |

```sh
pio run -e bluepill_blink_hid -t upload
pio device monitor -e bluepill_blink_hid
```

## Структура

- `src/<окружение>/` — код окружения (выбирается через `build_src_filter`).
- `doc/<окружение>/` — документация: [bluepill_blink](doc/bluepill_blink/README.md).
- `prompts/` — промпты для Claude Code, по которым создавались окружения.
