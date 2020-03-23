#include <Arduino.h>
#include "gpio/nrf52_gpio.h"
#include "position.h"

typedef struct {
  uint32_t raising_time_ticks;
  uint32_t length_ticks;
} Pulse;

/************DEBUG************/
#define TOTAL_PULSES    20
uint32_t pulses[TOTAL_PULSES];  //in ticks
uint16_t pulsesCount = 0;
bool printed = false;
bool debug_micros = true;
bool coordsWithCaption = false;


#define TICK                      0.0625      //us
#define MICROSECS_IN_TICKS(x)     (x / TICK)
#define BASE_STATIONS_COUNT       2
#define CYCLES_COUNT              4

uint32_t cyclePeriodLengthInTicks = MICROSECS_IN_TICKS(8333);    // 8333 us
uint32_t centerCycleLengthInTicks = MICROSECS_IN_TICKS(4000);    // 4000 us
uint32_t sweepStartWindowInTicks = MICROSECS_IN_TICKS(1222);     // 1222 us
uint32_t sweepEndWindowsInTicks = MICROSECS_IN_TICKS(6777);      // 6777 us

bool cycleSynchronized = false;
Pulse syncPulses[BASE_STATIONS_COUNT][CYCLES_COUNT];
Pulse sweepPulses[CYCLES_COUNT];
uint8_t cycle = 0;    // 4 cycles (0 - 3)
uint8_t basestation_index = 0;
uint32_t resetCyclesCount = 0;

#define TOTAL_ANGLES        50
float32_t angles[TOTAL_ANGLES][4];
uint16_t anglesCount = 0;

float32_t anglesBuffer[4];
bool anglesProccesed = true;

/**
 * @brief Function returns integer number which expresses bits of
 *        caught pulse. Bits are follows Skip | Data | Axis.
 * 
 * @param pulseLengthTicks 
 * @return int8_t (0 - 7) for sync pulses, (< 0) for sweeps, (> 7) invalid
 */
int8_t decodePulse(int32_t pulseLengthTicks) {
  return (pulseLengthTicks - 889) / 165; 
}

void computeAngles(void) {
  float32_t angs[4];
  for(uint8_t c = 0; c < CYCLES_COUNT; c++) {
    if(sweepPulses[c].raising_time_ticks == UINT32_MAX) { // sweep pulse was not detected
      return;
    }

    uint32_t time = sweepPulses[c].raising_time_ticks - syncPulses[c / 2][c].raising_time_ticks;
    if(time < sweepStartWindowInTicks || time > sweepEndWindowsInTicks) { // sweep pulse was in invalid range
      return;
    }
    
    angs[c] = ((int32_t) time - (int32_t) centerCycleLengthInTicks) * PI / (int32_t) cyclePeriodLengthInTicks;
  }

  if(anglesProccesed) {
    memcpy(anglesBuffer, angs, sizeof angs);
    anglesProccesed = false;
  }
  /*if(anglesCount < TOTAL_ANGLES) {
    memcpy(angles[anglesCount++], angs, sizeof angs);
  }*/
}

void resetPulses(void) {
  for(uint8_t b = 0; b < BASE_STATIONS_COUNT; b++) {
    for(uint8_t c = 0; c < CYCLES_COUNT; c++) {
      syncPulses[b][c].raising_time_ticks = UINT32_MAX;
      syncPulses[b][c].length_ticks = UINT32_MAX;
    }
  }
  memset(&sweepPulses, UINT32_MAX, sizeof sweepPulses);
}

void resetCycle(void) {
  resetCyclesCount++;
  resetPulses();
  cycle = 0;
  basestation_index = 0;
  cycleSynchronized = false;
}

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

      // debug purpose
      if(pulsesCount < TOTAL_PULSES) {
        pulses[pulsesCount++] = pulse_length_ticks;
      }

      int8_t bits = decodePulse((int32_t) pulse_length_ticks);
      uint8_t skip = 0;
      uint8_t data = 0;
      uint8_t axis = 0;
      
      if(bits < 0) {
        // process sweep
        if(cycleSynchronized && sweepPulses[cycle].raising_time_ticks == UINT32_MAX) { // sometime two short pulses occur consecutively
          sweepPulses[cycle].raising_time_ticks = raising_time;
          sweepPulses[cycle].length_ticks = pulse_length_ticks;
        }
        return;
      }

      if(bits > 7) {
        // invalid length of pulse
        resetCycle();
        return;
      }

      // extract bits
      skip = ((uint8_t) bits >> 2) & 1;
      data = ((uint8_t) bits >> 1) & 1;
      axis = ((uint8_t) bits) & 1;

      // find start of full cycle - situation where master basestation sweeps room in axis 0
      if(!cycleSynchronized) {
        if(!skip && !axis) {
          // candidate for master's lighthouse sync pulse in axis 0 (start of cycle)
          syncPulses[basestation_index][cycle].raising_time_ticks = raising_time;
          syncPulses[basestation_index][cycle].length_ticks = pulse_length_ticks;
          basestation_index++;
        } else {
          if(basestation_index == 1) {
            // candidate pulse was detected
            uint32_t diff = raising_time - syncPulses[basestation_index - 1][cycle].raising_time_ticks;
            if(diff >= MICROSECS_IN_TICKS(400) && diff <= MICROSECS_IN_TICKS(420)) {
              // start of cycle was detected
              syncPulses[basestation_index][cycle].raising_time_ticks = raising_time;
              syncPulses[basestation_index][cycle].length_ticks = pulse_length_ticks;
              basestation_index++;
              cycleSynchronized = true;
              // TODO: (maybe not necessary)
              // set value of CC[2] register to MICROSECS_IN_TICKS(8333 * 4) - falling_time
              // then set value of CC[2] register to MICROSECS_IN_TICKS(8333 * 4) at the start of every cycle

              return;
            } else {
              // start was not detected, reset sync pulses counter
              basestation_index = 0;
            }
          }
        }
      }

      if(!cycleSynchronized) return;

      if(basestation_index == 2) {
        cycle++;
        basestation_index = 0;
      }

      if(cycle == 4) {
        // start of new cycle
        cycle = 0;
        computeAngles();
        resetPulses();
      }

      // check sync pulses and if necessary reset cycle
      /*switch(cycle) {
        case 0:
          if(basestation_index == 0 && !skip && !axis) {
            // master lighthouse [Axis 0]
          } else if(basestation_index == 1 && skip) {
            // slave lighthouse  [Skip]
          } else {
            // invalid
            resetCycle();
          }
          break;
        case 1:
          if(basestation_index == 0 && !skip && axis) {
            // master lighthouse [Axis 1]
          } else if(basestation_index == 1 && skip) {
            // slave lighthouse  [Skip]
          } else {
            // invalid
            resetCycle();
          }
          break;
        case 2:
          if(basestation_index == 0 && skip) {
            // master lighthouse [Skip]
          } else if(basestation_index == 1 && !skip && !axis) {
            // slave lighthouse  [Axis 0]
          } else {
            // invalid
            resetCycle();
          }
          break;
        case 3:
          if(basestation_index == 0 && skip) {
            // master lighthouse [Skip]
          } else if(basestation_index == 1 && !skip && axis) {
            // slave lighthouse  [Axis 1]
          } else {
            // invalid
            resetCycle();
          }
          break;
      }*/

      syncPulses[basestation_index][cycle].raising_time_ticks = raising_time;
      syncPulses[basestation_index][cycle].length_ticks = pulse_length_ticks;

      basestation_index++;


      //NRF_TIMER1->TASKS_CLEAR = 1;    // clear timer at the end of cycle - 8333 us
    }
  }
}

void setup() {
  Serial.setPins(A_PIN_UART_RX, A_PIN_UART_TX);
  Serial.begin(115200);

  resetPulses();

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
  NRF_PPI->CHEN = ((uint32_t) PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos)
                  | ((uint32_t) PPI_CHEN_CH1_Enabled << PPI_CHEN_CH1_Pos);

  // start TIMER1
  NRF_TIMER1->TASKS_START = 1;
}

void loop() {
  if(!anglesProccesed) {
    vec3d xyz;
    bool res = calcPosition(anglesBuffer, xyz);
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
    anglesProccesed = true;
  }
  /*if(anglesCount == TOTAL_ANGLES && !printed) {
    for(uint16_t i = 0; i < TOTAL_ANGLES; i++) {
      for(uint8_t a = 0; a < 4; a++) {
        Serial.print(angles[i][a]);
        Serial.print(" ");
      }
      Serial.println();
    }
    printed = true;
  }*/
  /*if(pulsesCount == TOTAL_PULSES && !printed) {   
    for(uint16_t i = 0; i < pulsesCount; i++) {
        if(debug_micros) {
          Serial.print(pulses[i] * 0.0625);
        } else {
          Serial.print(pulses[i]);
        }  
        Serial.print(" ");
    }
    Serial.println();
    printed = true;
  }*/
}