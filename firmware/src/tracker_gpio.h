#pragma once

#include <nrf.h>
#include <wiring_constants.h>
#include <Arduino.h>

#define NRF_P1_BASE                     0x50000300UL
#define NRF_P1                          ((NRF_GPIO_Type           *) NRF_P1_BASE)

#define PIN_RED_LED     GPIO_PIN {1, 11}  //P1.11
#define PIN_GREEN_LED   GPIO_PIN {1, 10}  //P1.10
#define PIN_BLUE_LED    GPIO_PIN {0, 3}   //P0.03

#define PIN_BTN1        GPIO_PIN {1, 13}  //P1.13

#define PIN_E_TS4231    GPIO_PIN {0, 13}  //P0.13
#define PIN_D_TS4231    GPIO_PIN {0, 24}  //P0.24

#define PIN_AI3         GPIO_PIN {0, 5}   //P0.05        
#define PIN_AI4         GPIO_PIN {0, 28}  //P0.28
#define PIN_AI5         GPIO_PIN {0, 29}  //P0.29
#define PIN_AI6         GPIO_PIN {0, 30}  //P0.30

#define PIN_RST         GPIO_PIN {0, 18}  //P0.18
//#define PIN_SWD         GPIO_PIN {0, 0}   //SWDIO
//#define PIN_SWC         GPIO_PIN {0, 0}   //SWDCLK


#define A_PIN_AI3           20          
#define A_PIN_AI4           16
#define A_PIN_AI5           17
#define A_PIN_AI6           18

#define A_PIN_E_TS4231      2
#define A_PIN_D_TS4231      12

#define A_PIN_UART_TX       A_PIN_AI4       //AI4
#define A_PIN_UART_RX       PIN_SERIAL_RX   //DEFAULT RX SERIAL PIN


typedef struct {
    uint8_t port;
    uint32_t number;
} GPIO_PIN;

/**
 * \brief Configures the specified pin to behave either as an input or an output.
 *
 * \param pin The pin whose mode you wish to set
 * \param mode Can be INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN
 */
extern void pinMode(GPIO_PIN pin, uint32_t mode);


/**
 * \brief Write a HIGH or a LOW value to a digital pin.
 *
 * If the pin has been configured as an OUTPUT with pinMode(), its voltage will be set to the
 * corresponding value: 5V (or 3.3V on 3.3V boards) for HIGH, 0V (ground) for LOW.
 *
 * If the pin is configured as an INPUT, writing a HIGH value with digitalWrite() will enable an internal
 * 20K pullup resistor (see the tutorial on digital pins). Writing LOW will disable the pullup. The pullup
 * resistor is enough to light an LED dimly, so if LEDs appear to work, but very dimly, this is a likely
 * cause. The remedy is to set the pin to an output with the pinMode() function.
 *
 * \note Digital pin PIN_LED is harder to use as a digital input than the other digital pins because it has an LED
 * and resistor attached to it that's soldered to the board on most boards. If you enable its internal 20k pull-up
 * resistor, it will hang at around 1.7 V instead of the expected 5V because the onboard LED and series resistor
 * pull the voltage level down, meaning it always returns LOW. If you must use pin PIN_LED as a digital input, use an
 * external pull down resistor.
 *
 * \param pin the pin
 * \param value HIGH or LOW
 */
extern void digitalWrite(GPIO_PIN pin, uint32_t value);

/**
 * \brief Reads the value from a specified digital pin, either HIGH or LOW.
 *
 * \param pin The digital pin you want to read (int)
 *
 * \return HIGH or LOW
 */
extern int digitalRead(GPIO_PIN pin);
