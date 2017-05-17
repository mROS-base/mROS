#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "blinky.h"
#include "SoftPWM.h"

#define LED_ON      1
#define LED_OFF     0

#define TIME_10ms   1
#define TIME_20ms   2
#define TIME_30ms   3
#define TIME_40ms   4
#define TIME_50ms   5
#define TIME_100ms  10
#define TIME_200ms  20
#define TIME_500ms  50

static Ticker flipper;                                          // Tick Timer

static DigitalOut ledu(P6_12);                                  // LED-User
static SoftPWM ledr(P6_13);                                     // LED-Red
static SoftPWM ledg(P6_14);                                     // LED-Green
static SoftPWM ledb(P6_15);                                     // LED-Blue

static unsigned int syscnt_u;                                   // SystemCounter for LED-User
static unsigned int syscnt_r;                                   // SystemCounter for LED-Red
static unsigned int syscnt_g;                                   // SystemCounter for LED-Green
static unsigned int syscnt_b;                                   // SystemCounter for LED-Blue

static unsigned int flg_ledu;                                   // LED-User Flag
static unsigned int flg_ledr;                                   // LED-Red Flag
static unsigned int flg_ledg;                                   // LED-Green Flag
static unsigned int flg_ledb;                                   // LED-Blue Flag

void flip() {
	// Check 500ms for LED-User
    syscnt_u++;                                                 // increment SystemCounter for LED-User
    if( syscnt_u >= TIME_500ms ){
        flg_ledu++;
        syscnt_u = 0;
    }

    // Check 10ms for LED-Red
    syscnt_r++;                                                 // increment SystemCounter for LED-Red
    if( syscnt_r >= TIME_10ms ){
        flg_ledr++;
        syscnt_r = 0;
    }

    // Check 20ms for LED-Green
    syscnt_g++;                                                 // increment SystemCounter for LED-Green
    if( syscnt_g >= TIME_20ms ){
        flg_ledg++;
        syscnt_g = 0;
    }

    // Check 30ms for LED-Blue
    syscnt_b++;                                                 // increment SystemCounter for LED-Blue
    if( syscnt_b >= TIME_30ms ){
        flg_ledb++;
        syscnt_b = 0;
    }
}

void blinky_main_task(intptr_t exinf) {	
	ledu = LED_OFF;                                             // LED-User Off
    ledr.period_ms(10);                                         // Set PWM Period 10ms
	
    ledr = 0.0f;                                                // Set LED-Red Duty

    ledg.period_ms(10);                                         // Set PWM Period 10ms
    ledg = 0.0f;                                                // Set LED-Green Duty

    ledb.period_ms(10);                                         // Set PWM Period 10ms
    ledb = 0.0f;                                                // Set LED-Blue Duty

    flg_ledu = 0;                                               // Initialize LED-User Flag
    flg_ledr = 0;                                               // Initialize LED-Red Flag
    flg_ledg = 0;                                               // Initialize LED-Green Flag
    flg_ledb = 0;                                               // Initialize LED-Blue Flag

    syscnt_u = 0;                                               // Initialize System Counter for LED-User
    syscnt_r = 0;                                               // Initialize System Counter for LED-Red
    syscnt_g = 0;                                               // Initialize System Counter for LED-Green
    syscnt_b = 0;                                               // Initialize System Counter for LED-Blue

    unsigned int cntr = 0;                                      // Initialize LED-Red Counter
    unsigned int cntg = 0;                                      // Initialize LED-Green Counter
    unsigned int cntb = 0;                                      // Initialize LED-Blue Counter

    int cntrd = 1;                                              // Set LED-Red Counter Direction +1
    int cntgd = 1;                                              // Set LED-Green Counter Direction +1
    int cntbd = 1;                                              // Set LED-Blue Counter Direction +1

    unsigned int flg_ledu_last = 0;                             // Initialize LED-User Flag  (last value)
    unsigned int flg_ledr_last = 0;                             // Initialize LED-Red Flag   (last value)
    unsigned int flg_ledg_last = 0;                             // Initialize LED-Green Flag (last value)
    unsigned int flg_ledb_last = 0;                             // Initialize LED-Blue Flag  (last value)

	flipper.attach_us(&flip, 10000);                            // TickerTime Set 10ms

    while(1) {

//----- LED User -----
        if(flg_ledu_last != flg_ledu) {                         // Has LED-User Flag been Changed?
            flg_ledu_last = flg_ledu;                           // Save current value
            ledu =!ledu;                                        // Invert LED-User
        }

//----- LED Red -----
        if(flg_ledr_last != flg_ledr) {                         // Has LED-Red Flag been Changed?
            flg_ledr_last = flg_ledr;                           // Save current value
 
            if(cntr ==   0) cntrd =  1;                         // Set Direction(+1)
            if(cntr >= 127) cntrd = -1;                         // Set Direction(-1)
            cntr += cntrd;                                      // Increment/Decrement Counter
            ledr = (float)cntr / 128;                           // Set LED-Red Duty
        }

//----- LED Green -----
        if(flg_ledg_last != flg_ledg) {                         // Has LED-Green Flag been Changed?
            flg_ledg_last = flg_ledg;                           // Save current value
 
            if(cntg ==   0) cntgd =  1;                         // Set Direction(+1)
            if(cntg >= 127) cntgd = -1;                         // Set Direction(-1)
            cntg += cntgd;                                      // Increment/Decrement Counter
            ledg = (float)cntg / 128;                           // Set LED-Green Duty
        }

//----- LED Blue -----
        if(flg_ledb_last != flg_ledb) {                         // Has LED-Blue Flag been Changed?
            flg_ledb_last = flg_ledb;                           // Save current value
 
            if(cntb ==   0) cntbd =  1;                         // Set Direction(+1)
            if(cntb >= 127) cntbd = -1;                         // Set Direction(-1)
            cntb += cntbd;                                      // Increment/Decrement Counter
            ledb = (float)cntb / 128;                           // Set LED-Blue Duty
        }
	}

	//	return 0;
}
