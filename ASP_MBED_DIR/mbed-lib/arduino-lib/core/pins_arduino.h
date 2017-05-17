/* pins_arduino.h */
/* Copyright (C) 2016 Nozomu Fujita, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#ifndef GRPEACH
#error "unknown target."
#endif/*GRPEACH*/

#define NUM_DIGITAL_PINS            89

/** IO pins, header CN14. */
#define PIN_IO0     0 // P2_15(IRQ1)
#define PIN_IO1     1 // P2_14(IRQ0)
#define PIN_IO2     2 // P4_7
#define PIN_IO3     3 // P4_6
#define PIN_IO4     4 // P4_5
#define PIN_IO5     5 // P4_4
#define PIN_IO6     6 // P8_13
#define PIN_IO7     7 // P8_11

/** IO pins, header CN9. */
#define PIN_IO8     8 // P8_15
#define PIN_IO9     9 // P8_14
#define PIN_IO10   10 // P10_13
#define PIN_IO11   11 // P10_14
#define PIN_IO12   12 // P10_15
#define PIN_IO13   13 // P10_12

/** IO pins, header CN15. */
#define PIN_IO14   14 // P1_8(IRQ2)
#define PIN_IO15   15 // P1_9(IRQ3)
#define PIN_IO16   16 // P1_10(IRQ4)
#define PIN_IO17   17 // P1_11(IRQ5)
#define PIN_IO18   18 // P1_13
#define PIN_IO19   19 // P1_15

/** IO pins, header CN16. */
#define PIN_IO20   20 // P10_0
#define PIN_IO21   21 // P1_1(IRQ1)

/** IO pins, header CN8. */
#define PIN_IO22   22 // P5_7
#define PIN_IO23   23 // P5_6(IRQ6)
#define PIN_IO24   24 // P5_5
#define PIN_IO25   25 // P5_4
#define PIN_IO26   26 // P5_3
#define PIN_IO27   27 // P5_2
#define PIN_IO28   28 // P5_1
#define PIN_IO29   29 // P5_0

/** IO pins, header CN13. */
#define PIN_IO30   30 // P7_15
#define PIN_IO31   31 // P8_1
#define PIN_IO32   32 // P2_9
#define PIN_IO33   33 // P2_10
//#define PIN_IO34   34 // NC
//#define PIN_IO35   35 // NC

/** IO pins, header CN11. */
#define PIN_IO36   36 // P2_0(IRQ5)
#define PIN_IO37   37 // P2_1
#define PIN_IO38   38 // P2_2
#define PIN_IO39   39 // P2_3
#define PIN_IO40   40 // P2_4
#define PIN_IO41   41 // P2_5
#define PIN_IO42   42 // P2_6
#define PIN_IO43   43 // P2_7

/** IO pins, header CN12. */
#define PIN_IO44   44 // P3_8
#define PIN_IO45   45 // P3_9(IRQ6)
#define PIN_IO46   46 // P3_10
#define PIN_IO47   47 // P3_11
#define PIN_IO48   48 // P3_12
#define PIN_IO49   49 // P3_13
#define PIN_IO50   50 // P3_14
#define PIN_IO51   51 // P3_15

/** IO pins, header CN16. */
#define PIN_IO52   52 // P1_0(IRQ0)
#define PIN_IO53   53 // P1_12
#define PIN_IO54   54 // P1_7(IRQ7)
#define PIN_IO55   55 // P1_6(IRQ6)

/** IO pins, header CN20. */
#define PIN_IO56   56 // P6_6
#define PIN_IO57   57 // P7_7
#define PIN_IO58   58 // P8_7(IRQ5)
#define PIN_IO59   59 // P7_6
#define PIN_IO60   60 // P6_5
#define PIN_IO61   61 // P6_4(IRQ3)
#define PIN_IO62   62 // P6_8(IRQ0)

/** IO pins, header CN19. */
#define PIN_IO63   63 // P7_5
#define PIN_IO64   64 // P7_4
#define PIN_IO65   65 // P6_7
#define PIN_IO66   66 // P7_2

/** IO pins, header CN9. */
#define PIN_IO67   67 // P1_3(IRQ3)
#define PIN_IO68   68 // P1_2(IRQ2)
#define PIN_I2C_SDA PIN_IO67
#define PIN_I2C_SCL PIN_IO68

/** IO pins, header CN8. */
#define PIN_IO69   69 // P2_13(IRQ7)
#define PIN_IO70   70 // P4_0

/** IO pins, header CN5. */
#define PIN_IO71   71 // P8_3(IRQ1)
#define PIN_IO72   72 // P8_4
#define PIN_IO73   73 // P8_5
#define PIN_IO74   74 // P8_6
#define PIN_IO75   75 // P7_8(IRQ1)

/** IO pins, header CN9. */
#define PIN_IO76   76 // P3_2
#define PIN_IO77   77 // P8_8

/** IO pins, header CN17. */
#define PIN_IO78   78 // P11_12(IRQ3)
#define PIN_IO79   79 // P11_13
#define PIN_IO80   80 // P11_14
#define PIN_IO81   81 // P11_15(IRQ1)

/** USB TX and RX. */
#define PIN_USBTX  82 // P6_3(IRQ2)
#define PIN_USBRX  83 // P6_2(IRQ7)

/** LEDs (D1-D4) and SW. */
#define PIN_LED0   87 // P6_12(=LED4(=LED_USER))
#define PIN_LED1   84 // P6_13(=LED1(=LED_RED))
#define PIN_LED2   85 // P6_14(=LED2(=LED_GREEN))
#define PIN_LED3   86 // P6_15(=LED3(=LED_BLUE))
#define PIN_LED4   87 // P6_12(=LED4(=LED_USER))
#define PIN_SW     88 // P6_0(IRQ5)(=USER_BUTTON0)
#define PIN_LED_RED     PIN_LED1
#define PIN_LED_GREEN   PIN_LED2
#define PIN_LED_BLUE    PIN_LED3
#define PIN_LED_USER    PIN_LED4
#define PIN_USER_BUTTON0 PIN_SW

// ANALOG IO PINS -------------------------------------------------------------/

/** Analog pins, header CN15. */
#define PIN_AN000     14
#define PIN_AN001     15
#define PIN_AN002     16
#define PIN_AN003     17
#define PIN_AN004     18
#define PIN_AN005     19

// I2C PINS -------------------------------------------------------------------/

/** I2C pins, header CN15. */
static const uint8_t SDA = PIN_IO18;
static const uint8_t SCL = PIN_IO19;

// SPI PINS -------------------------------------------------------------------/

/** SPI pins, header CN9. */
static const uint8_t SS   = PIN_IO10; // PIN_IO22 is used for SD as SS
static const uint8_t MOSI = PIN_IO11;
static const uint8_t MISO = PIN_IO12;
static const uint8_t SCK  = PIN_IO13;

#endif/*Pins_Arduino_h*/
