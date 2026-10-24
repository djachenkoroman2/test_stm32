# Окружение bluepill_blink

## Назначение

Быстрая проверка платы Blue Pill (STM32F103C8T6): прошивка заливается, MCU стартует, тактирование работает — встроенный светодиод мигает раз в секунду.

Код: `src/bluepill_blink/main.cpp`. Окружение в `platformio.ini`: `[env:bluepill_blink]`.

## Светодиод

Встроенный светодиод подключён к **PC13** между пином и +3.3V (через резистор). Поэтому он **активен низким уровнем**: `LOW` — горит, `HIGH` — погашен.

Период мигания — 1 с (500 мс горит / 500 мс не горит), задаётся константой `BLINK_HALF_PERIOD_MS`. Мигание сделано на `millis()` без блокирующего `delay()`.

При каждом переключении в `Serial` (115200 бод) выводится `LED ON` / `LED OFF`. В stm32duino без включённого USB CDC `Serial` — это USART1: **PA9 (TX)**, **PA10 (RX)**. Чтобы увидеть вывод, нужен USB-UART адаптер.

## Команды

```sh
pio run -e bluepill_blink                # сборка
pio run -e bluepill_blink -t upload      # прошивка
pio device monitor -e bluepill_blink     # монитор порта
```

## Прошивка через ST-Link (по умолчанию)

| ST-Link | Blue Pill |
|---------|-----------|
| 3.3V    | 3.3       |
| GND     | GND       |
| SWDIO   | DIO (PA13)|
| SWCLK   | DCLK (PA14)|

Перемычки BOOT0 и BOOT1 — в положении 0.

## Альтернатива: прошивка через USB-UART

1. В `[env:bluepill_blink]` заменить `upload_protocol = stlink` на `upload_protocol = serial`.
2. Подключить адаптер: TX адаптера → PA10, RX адаптера → PA9, GND → GND, питание 3.3V.
3. Установить перемычку **BOOT0 = 1**, нажать RESET, выполнить `pio run -e bluepill_blink -t upload`.
4. Вернуть **BOOT0 = 0** и нажать RESET, чтобы запустить прошивку.

## Прошивка по USB через HID Bootloader

Окружение `[env:bluepill_blink_hid]` собирает тот же код, но:

- заливает прошивку по USB через [HID Bootloader](https://github.com/Serasidis/STM32_HID_Bootloader) (`upload_protocol = hid`);
- PlatformIO сам размещает прошивку с адреса `0x08000800`, сразу за загрузчиком (2 КБ);
- `Serial` идёт через USB CDC: плата видна как `/dev/ttyACM*`, отдельный USB-UART не нужен.

### 1. Собрать утилиту hid-flash (один раз)

PlatformIO её не поставляет, а ищет в `PATH`.

```sh
sudo apt install libusb-1.0-0-dev
git clone https://github.com/Serasidis/STM32_HID_Bootloader.git
make -C STM32_HID_Bootloader/cli
mkdir -p ~/.local/bin && cp STM32_HID_Bootloader/cli/hid-flash ~/.local/bin/
```

Доступ к устройству без root (VID:PID загрузчика — `1209:BEBA`):

```sh
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="beba", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/49-hid-bootloader.rules
sudo udevadm control --reload-rules
```

### 2. Записать загрузчик (один раз, через ST-Link)

Файл для Blue Pill — `hid_generic_pc13.bin` из папки F103 в архиве
[релиза](https://github.com/Serasidis/STM32_HID_Bootloader/releases).

```sh
~/.platformio/packages/tool-openocd/bin/openocd \
  -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program hid_generic_pc13.bin 0x08000000 verify reset exit"
```

> Прошивка окружения `bluepill_blink` через ST-Link пишет с адреса `0x08000000` и **затирает загрузчик**. После этого его нужно записывать заново.

### 3. Заливать прошивку по USB

```sh
pio run -e bluepill_blink_hid -t upload
pio device monitor -e bluepill_blink_hid
```

`hid-flash` через USB CDC перезагружает плату в загрузчик. Если плата не переключилась (например, загрузчик только что записан и прошивки ещё нет, или прошивка зависла), войдите в загрузчик вручную: перемычку **BOOT1 = 1**, RESET, залить прошивку, затем вернуть **BOOT1 = 0**.

### Если заливка не проходит

- `Unable to open the [1209:BEBA] device` — нет прав на устройство: не установлено правило udev из шага 1 или плату не переподключили после его установки.
- `Error while sending <reset pages> command` (в отладке libusb — ошибка `-71`) — загрузчик завис, например после прерванной заливки. Нажмите RESET или переподключите USB.
- В `hid-flash` v2.2.1 есть баг: если устройство не открылось, утилита падает с segfault вместо сообщения об ошибке. В `cli/main.c` условие `if (i == 10 && handle != NULL)` нужно заменить на `if (handle == NULL)`.

### Если компьютер не видит плату

На многих Blue Pill резистор R10 (подтяжка D+) стоит 10 кОм или 4.7 кОм вместо 1.5 кОм. Замените его на 1.5 кОм или добавьте 1.8 кОм между **PA12** и **3.3V**.

## Тест пройден, если

- светодиод на PC13 мигает с периодом ~1 с;
- (опционально) в мониторе порта раз в 500 мс чередуются строки `LED ON` / `LED OFF`.
