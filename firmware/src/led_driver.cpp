#include "led_driver.h"

void LedDriver::showFailed(void) {
    if(showingState == STATE_FAILED) {
        return;
    }

    reset();
    showingState = STATE_FAILED;
}

void LedDriver::showGeometryNotSet(void) {
    if(showingState == STATE_GEOMETRY_NOT_SET) {
        return;
    }

    reset();
    showingState = STATE_GEOMETRY_NOT_SET;
}

void LedDriver::showTracking(void) {
    if(showingState == STATE_TRACKING) {
        return;
    }

    reset();
    showingState = STATE_TRACKING;
}

void LedDriver::showTrackingTimeout(void) {
    if(showingState == STATE_TRACKING_TIMEOUT) {
        return;
    }

    reset();
    showingState = STATE_TRACKING_TIMEOUT;
}

void LedDriver::turnAllOf(void) {
    reset();
}

void LedDriver::loop(void) {
    const LedTimings *ledTiming;
    uint8_t totalTimings;
    switch(showingState) {
        case STATE_ALL_OFF:
            return;
        case STATE_FAILED:
            ledTiming = failedTimings;
            totalTimings = sizeof(failedTimings) / sizeof(*ledTiming);
            break;
        case STATE_TRACKING:
            ledTiming = trackingTimings;
            totalTimings = sizeof(trackingTimings) / sizeof(*ledTiming);
            break;
        case STATE_TRACKING_TIMEOUT:
            ledTiming = trackingTimeoutTimings;
            totalTimings = sizeof(trackingTimeoutTimings) / sizeof(*ledTiming);
            break;
        case STATE_GEOMETRY_NOT_SET:
            ledTiming = geometryNotSetTimings;
            totalTimings = sizeof(geometryNotSetTimings) / sizeof(*ledTiming);
            break;
        default:
            return;
    }

    uint32_t currentMillis = millis();
    if(!startMillis) {
        // the start of whole cycle
        startMillis = currentMillis;
        isLedTurnOn = true;
    }

    if(isLedTurnOn) {
        // LED is turned on
        digitalWrite(ledTiming[timingsIndex].led, LOW);
        if(currentMillis - startMillis > ledTiming[timingsIndex].turnOn) {
            // turn off LED
            isLedTurnOn = false;
            startMillis = currentMillis;
            digitalWrite(ledTiming[timingsIndex].led, HIGH);
        }
    } else {
        // LED is turned off
        digitalWrite(ledTiming[timingsIndex].led, HIGH);
        if(currentMillis - startMillis > ledTiming[timingsIndex].turnOff) {
            // turn on LED
            isLedTurnOn = true;
            startMillis = currentMillis;
            //end of cycle -> move index to next led timings
            timingsIndex = ++timingsIndex % totalTimings;
            digitalWrite(ledTiming[timingsIndex].led, LOW);
        }
    }
}

void LedDriver::setup(void) {
    reset();
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_BLUE_LED, OUTPUT);
}

void LedDriver::reset(void) {
    showingState = STATE_ALL_OFF;
    startMillis = 0;
    isLedTurnOn = false;
    timingsIndex = 0;
    digitalWrite(PIN_RED_LED, HIGH);
    digitalWrite(PIN_GREEN_LED, HIGH);
    digitalWrite(PIN_BLUE_LED, HIGH);
}