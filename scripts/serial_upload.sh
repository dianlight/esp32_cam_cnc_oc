#!/bin/bash

"~/.platformio/penv/bin/python" "~/.platformio/packages/tool-esptoolpy@1.20600.0/esptool.py" --chip esp32 --port "/dev/cu.usbserial-00000000" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 .pio/build/esp32cam/bootloader.bin 0x8000 .pio/build/esp32cam/partitions.bin 0x10000 .pio/build/esp32cam/firmware.bin
"~/.platformio/penv/bin/python" "~/.platformio/packages/tool-esptoolpy@1.20600.0/esptool.py" --chip esp32 --port "/dev/cu.usbserial-00000000" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 .pio/build/esp32cam/bootloader.bin 0x8000 .pio/build/esp32cam/partitions.bin 0x1F0000 .pio/build/esp32cam/firmware.bin
