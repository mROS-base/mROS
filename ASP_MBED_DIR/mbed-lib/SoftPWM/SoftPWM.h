#ifndef SoftPWM_H
#define SoftPWM_H
#define POSITIVE true
#define NEGATIVE false

#include "mbed.h"

class SoftPWM  
{
private:
    Timeout _timeout;
    Ticker _ticker;
    void end();
    DigitalOut pulse;
    bool positive;
    void TickerInterrapt();
    float width;
    float interval;
public:
    SoftPWM(PinName,bool mode=true); 
//    void attach_us(int);
    void start();
    void write(float);
    float read();
    void pulsewidth(float);
    void pulsewidth_ms(int);
    void pulsewidth_us(int);
    void period(float);
    void period_ms(int);
    void period_us(int);
    void stop();
    operator float()  { 
        if ( width <= 0.0 ) return 0.0;
        if ( width > 1.0 )  return 1.0;
        return width / interval;
    }
    SoftPWM& operator=(float duty)  {
        width = interval * duty;
        if ( duty <= 0.0 ) width =  0.0;
        if ( duty > 1.0 )  width =  interval;
        return *this;
    }
                
};
#endif