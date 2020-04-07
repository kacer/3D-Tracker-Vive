#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "tracker_gpio.h"
#include "position_calculator.h"
#include "pulse_processor.h"
#include "bluetooth.h"
#include "led_driver.h"

/************DEBUG************/
BaseStation baseStation0 = {
    {-0.462107, -0.199174, 0.864169,
     0.002075, 0.974207, 0.225645,
      -0.886822, 0.106065, -0.449775}, // rotation matrix by rows
    {1.548505, 1.168343, -1.145540}  // origin
};

BaseStation baseStation1 = {
    {-0.741860, 0.160941, -0.650955,
     -0.007757, 0.968645, 0.248326,
      0.670510, 0.189273, -0.717351}, // rotation matrix by rows
    {-0.862957, 1.201664, -1.565771}  // origin
};

/*************ABOVE THIS REMOVE!***************/

void anglesCb(float32_t *angles);
void baseStationGeometryCB(BaseStation *baseStations);

PulseProcessor pulseProcessor(anglesCb);
PositionCalculator positionCalculator = PositionCalculator();
Bluetooth* bluetooth = Bluetooth::getInstance();
LedDriver ledDriver = LedDriver();

const uint16_t MAX_POSITION_UPDATE_DELAY = 1000; // 1s
const uint16_t LIGHT_TIMEOUT = 500; // 500 ms
uint32_t lastPositionUpdateMillis;
bool receivingCoordinates = false;
bool lightDetected = false;


extern "C" {  //important; otherwise irq handler won't be called
    void GPIOTE_IRQHandler(void) {
        if(NRF_GPIOTE->EVENTS_IN[0] != 0) {
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            // raising edge
            lightDetected = true;
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

void anglesCb(float32_t *angles) {
    vec3d xyz;
    uint8_t result = positionCalculator.calcPosition(angles, xyz);
    
    switch(result) {
        case CALCULATION_SUCCESS:
            lastPositionUpdateMillis = millis();
            bluetooth->sendPosition(xyz);
            ledDriver.showTracking();
            receivingCoordinates = true;
            /*
            Serial.print(xyz[0], 6);
            Serial.print(" ");
            Serial.print(xyz[1], 6);
            Serial.print(" ");
            Serial.println(xyz[2], 6);
            */
            break;
        case BASESTATIONS_NOT_SET:
            ledDriver.showGeometryNotSet();
            receivingCoordinates = false;
            break;
        case CALCULATION_FAILED:
            ledDriver.showFailed();
            receivingCoordinates = false;
            break;
    }
}

void baseStationGeometryCB(BaseStation *baseStations) {
    positionCalculator.setBaseStationsGeometry(baseStations[0], baseStations[1]);
    // clear state showing that geometry is not set
    receivingCoordinates = true;
    ledDriver.showTracking();

    Bluetooth::getInstance()->printBytes((unsigned char*) baseStations[0].origin, sizeof(vec3d));
    Bluetooth::getInstance()->printBytes((unsigned char*) baseStations[0].rotationMatrix, sizeof(mat3Array));
    Bluetooth::getInstance()->printBytes((unsigned char*) baseStations[1].origin, sizeof(vec3d));
    Bluetooth::getInstance()->printBytes((unsigned char*) baseStations[1].rotationMatrix, sizeof(mat3Array));
}

bool waitForLight(uint16_t timeout) {
    uint32_t startTime = millis();
    
    while(millis() - startTime < timeout) {
        if(lightDetected) {
            return true;
        }
    }

    return false;
}

void setup() {
    Serial.setPins(A_PIN_UART_RX, A_PIN_UART_TX);
    Serial.begin(115200);

    bluetooth->setBaseStationGeometryCB(baseStationGeometryCB);
    bluetooth->setup();
    ledDriver.setup();

    // setting GPIOTE channels for raising and falling edge
    uint8_t pin_D = PIN_D_TS4231.number; 
    uint8_t port_pin_D = PIN_D_TS4231.port;
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

    if(!waitForLight(LIGHT_TIMEOUT)) {
        ledDriver.showFailed();
    }
}

void loop() {
    pulseProcessor.loop();
    bluetooth->poll();
    ledDriver.loop();
    
    if(receivingCoordinates && (millis() - lastPositionUpdateMillis) > MAX_POSITION_UPDATE_DELAY) {
        // tracker was occluded
        ledDriver.showTrackingTimeout();
    }
}