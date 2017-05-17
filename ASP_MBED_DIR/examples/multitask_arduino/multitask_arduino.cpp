#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "arduino_app.h"
#include "arduino_main.h"

/* GR-PEACH Sketch Template V1.00 */
#include <Arduino.h>

#define INTERVAL       100
#define INTERVAL_RED   100
#define INTERVAL_BLUE  150
#define INTERVAL_GREEN 200

void setup()
{
    pinMode(PIN_LED_RED   , OUTPUT);
    pinMode(PIN_LED_GREEN , OUTPUT);
    pinMode(PIN_LED_BLUE  , OUTPUT);
    pinMode(PIN_LED_USER  , OUTPUT);
	pinMode(PIN_SW        , INPUT);

	while(digitalRead(PIN_SW) == 0){
		digitalWrite(PIN_LED_USER, 1);
        delay(INTERVAL);
        digitalWrite(PIN_LED_USER, 0);
        delay(INTERVAL);
	}	
}

void loop()
{
	digitalWrite(PIN_LED_RED, 1);
	delay(INTERVAL_RED);
	digitalWrite(PIN_LED_RED, 0);
	delay(INTERVAL_RED);
}

void loop1(){
    digitalWrite(PIN_LED_GREEN, 1);
    delay(INTERVAL_GREEN);
    digitalWrite(PIN_LED_GREEN, 0);
	delay(INTERVAL_GREEN);
}

void loop2(){
    digitalWrite(PIN_LED_BLUE, 1);
    delay(INTERVAL_BLUE);
	digitalWrite(PIN_LED_BLUE, 0);
	delay(INTERVAL_BLUE);
}

/*
 *  Cyclic Handler
 * 
 *  This handler is called every 10 [ms] as specified in
 *  multitask_arduino.cfg.
 */
void cyclic_handler(intptr_t exinf) {
	irot_rdq(LOOP_PRI); /* change the running loop. */
}
