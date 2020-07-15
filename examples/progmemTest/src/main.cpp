#include <Arduino.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

const PROGMEM uint16_t _constWithAddr[] = { 10 };
const uint16_t _constWithoutAddr[] = { 1 };

int main() {
    printf("%d, %d", _constWithAddr[0], _constWithoutAddr[1]);
}
