#include "pulse_processor.h"

PulseProcessor::PulseProcessor(cb_angle pCallback) {
    angleCb = pCallback;
}

void PulseProcessor::loop(void) {
    Pulse pulse;
    if(!readPulse(pulse)) return;   // nothing to do

    int8_t bits = decodePulse(pulse);

    if(bits < 0) {
        // sweep pulse
        if(!cycleSynchronized) return;
        computeAngle(pulse);

        return;
    }

    if(bits > 7) {
        // invalid length of pulse
        resetCycle();
        
        return;
    }

    // extract bits
    uint8_t skip = ((uint8_t) bits >> 2) & 1;
    uint8_t data = ((uint8_t) bits >> 1) & 1;
    uint8_t axis = ((uint8_t) bits) & 1;

    // find start of full cycle - situation where master basestation sweeps room in axis 0
    if(!cycleSynchronized) {
        if(!skip && !axis) {
            // candidate for master's lighthouse sync pulse in axis 0 (start of cycle)
            startTime = pulse.raising_time_ticks;
            startCycleCandidate = true;
            baseStationIndex = 1;
            
            return;
        }
        if(startCycleCandidate) {
            // candidate pulse was detected
            uint32_t diff = pulse.raising_time_ticks - startTime;
            if(diff >= MICROSECS_IN_TICKS(400) && diff <= MICROSECS_IN_TICKS(420)) {
                // start of cycle was detected
                cycleSynchronized = true;
                baseStationIndex = 2;

                return;
            } else {
                // start was not detected, reset candidate
                startCycleCandidate = false;
            }
        }
    }
    if(!cycleSynchronized) return;

    if(baseStationIndex == 2) { // end of one cycle
        cycle++;
        baseStationIndex = 0;
    }
    if(cycle == CYCLES_COUNT) { // end of full cycle
        if(anglesCount == ANGLES_COUNT) { 
            angleCb(angles);   // let's propagate caught angles
        }
        cycle = 0;
        anglesCount = 0;
    }

    if((cycle / 2 == 0 && baseStationIndex == 0) ||
        (cycle / 2 == 1 && baseStationIndex == 1)) {
        startTime = pulse.raising_time_ticks;
    }
    baseStationIndex++;
}

void PulseProcessor::addPulse(Pulse pulse) {
    uint8_t w = (pulsesWriteIndex + 1) % PULSES_COUNT;
    uint8_t r = pulsesReadIndex % PULSES_COUNT;
    if(w == r) {
        // the data would be overwritten, let's reset cycle
        resetCycle();
        // reset read and write indexes too
        pulsesReadIndex = 0;
        pulsesWriteIndex = 0;
    }
    pulses[pulsesWriteIndex++ % PULSES_COUNT] = pulse;
}

bool PulseProcessor::readPulse(Pulse &pulse) {
    if(pulsesReadIndex == pulsesWriteIndex) {
        return false;
    }

    pulse = pulses[pulsesReadIndex++ % PULSES_COUNT];

    return true;
}

int8_t PulseProcessor::decodePulse(Pulse &pulse) {
    return ((int32_t) pulse.length_ticks - 889) / 165; 
}

void PulseProcessor::computeAngle(Pulse &pulse) {
    uint32_t time = (pulse.raising_time_ticks + pulse.length_ticks / 2) - startTime;
    if(time < sweepStartWindowInTicks || time > sweepEndWindowsInTicks) { // sweep pulse was in invalid range
        return;
    }

    angles[cycle] = ((int32_t) time - (int32_t) centerCycleLengthInTicks) * PI / (int32_t) cyclePeriodLengthInTicks;
    anglesCount++;
}

void PulseProcessor::resetCycle(void) {
    anglesCount = 0;
    cycle = 0;
    baseStationIndex = 0;
    cycleSynchronized = false;
    startCycleCandidate = false;
}