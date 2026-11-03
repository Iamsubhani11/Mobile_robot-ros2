#!/bin/bash
# Compiles the sketch for the Uno with the toolchain the Arduino IDE installed. usage: compile_uno.sh outdir
set -e
OUT=${1:-/tmp/amr_uno}; mkdir -p "$OUT"
A=$HOME/.arduino15/packages/arduino; BIN=$(ls -d $A/tools/avr-gcc/*)/bin; CORE=$A/hardware/avr/1.8.8
HERE=$(cd "$(dirname "$0")/../amr_firmware" && pwd)
F="-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -Os -ffunction-sections -fdata-sections -w"
I="-I$CORE/cores/arduino -I$CORE/variants/standard -I$CORE/libraries/Wire/src -I$CORE/libraries/Wire/src/utility -I$HERE"
objs=()
for f in $(find $CORE/cores/arduino $CORE/libraries/Wire/src -name '*.c' -o -name '*.cpp' -o -name '*.S'); do
  o="$OUT/$(echo "$f" | md5sum | cut -c1-10).o"; objs+=("$o")
  case "$f" in *.cpp) $BIN/avr-g++ -std=gnu++11 -fno-exceptions -fno-threadsafe-statics -fpermissive $F $I -c "$f" -o "$o";;
    *.c) $BIN/avr-gcc -std=gnu11 $F $I -c "$f" -o "$o";; *.S) $BIN/avr-gcc -x assembler-with-cpp $F $I -c "$f" -o "$o";; esac
done
$BIN/avr-g++ -std=gnu++11 -fno-exceptions -fno-threadsafe-statics $F -Wall $I -x c++ "$HERE/amr_firmware.ino" -c -o "$OUT/sketch.o"
$BIN/avr-gcc -Os -Wl,--gc-sections -mmcu=atmega328p -o "$OUT/amr.elf" "$OUT/sketch.o" "${objs[@]}" -lm
$BIN/avr-objcopy -O ihex -R .eeprom "$OUT/amr.elf" "$OUT/amr.hex"
$BIN/avr-size -C --mcu=atmega328p "$OUT/amr.elf" | grep -E 'Program|Data'
