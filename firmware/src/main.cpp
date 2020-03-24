#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "gpio/nrf52_gpio.h"
#include "position.h"
#include "pulse_processor.h"

/************DEBUG************/
#define TOTAL_PULSES    20
uint32_t pulses[TOTAL_PULSES];  //in ticks
uint16_t pulsesCount = 0;
bool printed = false;
bool debug_micros = true;
bool coordsWithCaption = false;

BLEPeripheral ledPeripheral = BLEPeripheral();

BLEService ledService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLECharCharacteristic ledCharacteristic = BLECharCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);


void anglesCb(float32_t *angles) {
  vec3d xyz;
    bool res = calcPosition(angles, xyz);
    if(res) {
      if(coordsWithCaption) {
        Serial.print("x: ");
        Serial.print(xyz[0], 3);
        Serial.print(" y: ");
        Serial.print(xyz[1], 3);
        Serial.print(" z: ");
        Serial.println(xyz[2], 3);
      } else {
        Serial.print(xyz[0], 6);
        Serial.print(" ");
        Serial.print(xyz[1], 6);
        Serial.print(" ");
        Serial.println(xyz[2], 6);
      }
    } else {
      Serial.println("Calculation was not successful");
    }
}

PulseProcessor pulseProcessor(anglesCb);

extern "C" {  //important; otherwise irq handler won't be called
  void GPIOTE_IRQHandler(void) {
    if(NRF_GPIOTE->EVENTS_IN[0] != 0) {
      NRF_GPIOTE->EVENTS_IN[0] = 0;
      // raising edge
    }

    if(NRF_GPIOTE->EVENTS_IN[1] != 0) {
      NRF_GPIOTE->EVENTS_IN[1] = 0;
      // falling edge
      uint32_t raising_time = NRF_TIMER1->CC[0];
      uint32_t falling_time = NRF_TIMER1->CC[1];
      uint32_t pulse_length_ticks = falling_time - raising_time;

      Pulse pulse = {raising_time, pulse_length_ticks};
      pulseProcessor.addPulse(pulse);
    }
  }
}

void setup() {
  Serial.setPins(A_PIN_UART_RX, A_PIN_UART_TX);
  Serial.begin(115200);

  pinMode(PIN_BLUE_LED, OUTPUT);
  digitalWrite(PIN_BLUE_LED, HIGH);
  ledPeripheral.setAdvertisedServiceUuid(ledService.uuid());
  ledPeripheral.addAttribute(ledService);
  ledPeripheral.addAttribute(ledCharacteristic);
  ledPeripheral.setLocalName("3D Tracker");
  ledPeripheral.begin();

  // setting GPIOTE channels for raising and falling edge
  uint8_t pin_D = 24;   //P0.24 real pin number, not index (12)
  uint8_t port_pin_D = 0;
  uint8_t gpiote_channel_raising = 0;
  uint8_t gpiote_channel_falling = 1;
  NRF_GPIOTE->CONFIG[gpiote_channel_raising] = ((uint32_t) GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
                                              | ((uint32_t) pin_D << GPIOTE_CONFIG_PSEL_Pos)
                                              | ((uint32_t) port_pin_D << 13UL)   // Bit PORT starts at 13, could be skipped
                                              | ((uint32_t) GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
  NRF_GPIOTE->CONFIG[gpiote_channel_falling] = ((uint32_t) GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
                                              | ((uint32_t) pin_D << GPIOTE_CONFIG_PSEL_Pos)
                                              | ((uint32_t) port_pin_D << 13UL)   // Bit PORT starts at 13, could be skipped
                                              | ((uint32_t) GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
  NRF_GPIOTE->INTENSET = ((uint32_t) GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos)
                        | ((uint32_t) GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos);
  NVIC_EnableIRQ(GPIOTE_IRQn);

  // config TIMER1 for capturing timestamps of rising and falling edges from GPIOTE
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER1->PRESCALER = 0;    // 16MHz
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

  // config PPI for transfering events of rising/falling edge on pin E to TIMER1 for capturing timestamps
  NRF_PPI->CH[gpiote_channel_raising].EEP = (uint32_t) &NRF_GPIOTE->EVENTS_IN[gpiote_channel_raising];
  NRF_PPI->CH[gpiote_channel_raising].TEP = (uint32_t) &NRF_TIMER1->TASKS_CAPTURE[gpiote_channel_raising];
  NRF_PPI->CH[gpiote_channel_falling].EEP = (uint32_t) &NRF_GPIOTE->EVENTS_IN[gpiote_channel_falling];
  NRF_PPI->CH[gpiote_channel_falling].TEP = (uint32_t) &NRF_TIMER1->TASKS_CAPTURE[gpiote_channel_falling];
  NRF_PPI->CHENSET = ((uint32_t) PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos)
                  | ((uint32_t) PPI_CHENSET_CH1_Enabled << PPI_CHENSET_CH1_Pos);

  // start TIMER1
  NRF_TIMER1->TASKS_START = 1;
}

void loop() {
  pulseProcessor.loop();
  
  BLECentral central = ledPeripheral.central();
  if (central) {
    //while (central.connected()) {
      if (ledCharacteristic.written()) {
        if (ledCharacteristic.value()) {
          digitalWrite(PIN_BLUE_LED, LOW);
        }
        else{
          digitalWrite(PIN_BLUE_LED, HIGH);
        }
      }
    //}
  }
}