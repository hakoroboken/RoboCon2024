#ifndef RMC_H_
#define RMC_H_

#include <Arduino.h>

#define M2006CURRENT 10000
#define M2006RPM 12000
#define M3508CURRENT 16384
#define M3508RPM 7000

namespace hakorobo2024
{
    class RoboMasterControl
    {
        public:
        RoboMasterControl(int max_rotation, int max_current);
        int16_t GetOutput(int target, int current);

        private:
        int maxRotation, maxCurrent, minRotation, minCurrent;
        int gain;
        int16_t prevOut;
    };
}

#endif