#include <Arduino.h>
#include "arm_math.h"

#define TICK                      0.0625      //us
#define MICROSECS_IN_TICKS(x)     (x / TICK)

#define CYCLES_COUNT              4
#define PULSES_COUNT              10          // ring buffer size
#define ANGLES_COUNT              4

typedef struct {
  uint32_t raising_time_ticks;
  uint32_t length_ticks;
} Pulse;

typedef void (*cb_angle)(float32_t *angles);

class PulseProcessor {
    public:
        void loop(void);
        void addPulse(Pulse pulse);
        PulseProcessor(cb_angle pCallback);

    private:
        Pulse pulses[PULSES_COUNT];
        uint32_t pulsesWriteIndex = 0;
        uint32_t pulsesReadIndex = 0;
        cb_angle angleCb;
        float32_t angles[ANGLES_COUNT];
        uint8_t anglesCount = 0;
        bool cycleSynchronized = false;
        bool startCycleCandidate = false;
        uint32_t startOfCycleTime;
        uint32_t startTime;     // time at rotor reached 0°
        uint8_t cycle = 0;      // cycle 0 - 3
        uint8_t baseStationIndex = 0;

        uint32_t cyclePeriodLengthInTicks = MICROSECS_IN_TICKS(8333);    // 8333 us
        uint32_t centerCycleLengthInTicks = MICROSECS_IN_TICKS(4000);    // 4000 us
        uint32_t sweepStartWindowInTicks = MICROSECS_IN_TICKS(1222);     // 1222 us
        uint32_t sweepEndWindowsInTicks = MICROSECS_IN_TICKS(6777);      // 6777 us


        /**
         * @brief Function returns integer number which expresses bits of
         *        caught pulse. Bits are follows Skip | Data | Axis.
         * 
         * @param pulse 
         * @return int8_t (0 - 7) for sync pulses, (< 0) for sweeps, (> 7) invalid
         */
        int8_t decodePulse(Pulse &pulse);
        
        /**
         * @brief Read pulses from ring buffer.
         * 
         * @param pulse read pulse if function returns true
         * @return true if read was successful
         */
        bool readPulse(Pulse &pulse);
        void computeAngle(Pulse &pulse);
        void resetCycle(void);
};