#include "tracker_gpio.h"

void pinMode(GPIO_PIN pin, uint32_t mode) {
    if(!(pin.port == 0 || pin.port == 1) || pin.number > 31) { // nRF52840 has P0 or P1 with pin 0 - 31
        return;
    }

    switch(mode) {
        case INPUT:
            if(pin.port == 0) {
                NRF_GPIO->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            } else {
                NRF_P1->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            }
            break;
        
        case INPUT_PULLUP:
            if(pin.port == 0) {
                NRF_GPIO->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            } else {
                NRF_P1->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            }
            break;

        case INPUT_PULLDOWN:
            if(pin.port == 0) {
                NRF_GPIO->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            } else {
                NRF_P1->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            }
            break;

        case OUTPUT:
            if(pin.port == 0) {
                NRF_GPIO->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            } else {
                NRF_P1->PIN_CNF[pin.number] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
            }
            break;
    }
}

void digitalWrite(GPIO_PIN pin, uint32_t value) {
    if(!(pin.port == 0 || pin.port == 1) || pin.number > 31) { // nRF52840 has P0 or P1 with pin 0 - 31
        return;
    }

    switch (value) {
        case LOW:
            if(pin.port == 0) {
                NRF_GPIO->OUTCLR = (1UL << pin.number);
            } else {
                NRF_P1->OUTCLR = (1UL << pin.number);
            }
            return;

        default:
            if(pin.port == 0) {
                NRF_GPIO->OUTSET = (1UL << pin.number);
            } else {
                NRF_P1->OUTSET = (1UL << pin.number);
            }
            return;
    }
}

int digitalRead(GPIO_PIN pin) {
    if(!(pin.port == 0 || pin.port == 1) || pin.number > 31) { // nRF52840 has P0 or P1 with pin 0 - 31
        return 0;
    }

    if(pin.port == 0) {
        return ((NRF_GPIO->IN >> pin.number) & 1UL) ? HIGH : LOW;
    } else {
        return ((NRF_P1->IN >> pin.number) & 1UL) ? HIGH : LOW;
    }
}