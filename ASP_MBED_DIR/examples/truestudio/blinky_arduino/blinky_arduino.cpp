#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "arduino_app.h"

/* GR-PEACH Sketch Template V1.00 */
#include <Arduino.h>

#define INTERVAL 100

void setup()
{
    pinMode(PIN_LED_RED   , OUTPUT);
    pinMode(PIN_LED_GREEN , OUTPUT);
    pinMode(PIN_LED_BLUE  , OUTPUT);
    pinMode(PIN_LED_USER  , OUTPUT);
	pinMode(PIN_SW        , INPUT);
}

void loop()
{
	while(digitalRead(PIN_SW) == 0){
		digitalWrite(PIN_LED_USER, 1);
        delay(INTERVAL);
        digitalWrite(PIN_LED_USER, 0);
        delay(INTERVAL);
	}
    digitalWrite(PIN_LED_RED, 1);
    delay(INTERVAL);
    digitalWrite(PIN_LED_RED, 0);
    digitalWrite(PIN_LED_GREEN, 1);
    delay(INTERVAL);
    digitalWrite(PIN_LED_GREEN, 0);
    digitalWrite(PIN_LED_BLUE, 1);
    delay(INTERVAL);
    digitalWrite(PIN_LED_BLUE, 0);
}
