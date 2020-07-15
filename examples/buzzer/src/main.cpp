#include <Arduino.h>

#include <avr/io.h>
#include <util/delay.h>

void setup() {
    DDRC |= (1 << DDC7);    // Make pin 13 be an output.
}

void loop () {
    PORTC |= (1 << PORTC7);   // Turn the LED on.
    _delay_ms(500);
    PORTC &= ~(1 << PORTC7);  // Turn the LED off.
    _delay_ms(500);
}

int main() {
    setup();

    while(1) loop();
}
