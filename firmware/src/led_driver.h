#include <Arduino.h>
#include "tracker_gpio.h"

const uint8_t STATE_ALL_OFF = 0;
const uint8_t STATE_FAILED = 1;
const uint8_t STATE_TRACKING = 2;
const uint8_t STATE_TRACKING_TIMEOUT = 3;
const uint8_t STATE_GEOMETRY_NOT_SET = 4;

typedef struct {
    uint16_t turnOn;    // ms
    uint16_t turnOff;   // ms
    GPIO_PIN led;
} LedTimings;

const LedTimings failedTimings[] = {{500, 1000, PIN_RED_LED}};
const LedTimings trackingTimings[] = {{60, 940, PIN_BLUE_LED}};
const LedTimings trackingTimeoutTimings[] = {{80, 920, PIN_RED_LED}};
const LedTimings geometryNotSetTimings[] = {{60, 160, PIN_GREEN_LED}, {160, 250, PIN_RED_LED}, {60, 1000, PIN_GREEN_LED}};

class LedDriver {
    public:
        void showFailed(void);
        void showGeometryNotSet(void);
        void showTracking(void);
        void showTrackingTimeout(void);
        void turnAllOf(void);
        void loop(void);
        void setup(void);
    private:
        uint8_t showingState;
        uint32_t startMillis;
        bool isLedTurnOn;
        uint8_t timingsIndex;

        void reset(void);
};