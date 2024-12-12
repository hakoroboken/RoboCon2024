#include "RMC.h"

namespace hakorobo2024
{
    RoboMasterControl::RoboMasterControl(int max_rotation, int max_current): maxRotation(max_rotation),maxCurrent(max_current)
    {
        minCurrent = -1 * maxCurrent;
        minRotation = -1 * maxRotation;

        gain = (float)(maxCurrent) * 0.1;
        prevOut = 0;
    }

    int16_t RoboMasterControl::GetOutput(int target, int actual)
    {
        if(abs(actual) < 1000)
        {
            if(target > 0)
            {
                prevOut = maxCurrent;
                return maxCurrent;
            }
            else if(target < 0)
            {
                prevOut = minCurrent;
                return minCurrent;
            }
            else
            {
                return 0;
            }
        }

        float target_p = (float)(target) * 1.2;
        float target_m = (float)(target) * 0.8;

        if(target_p > actual && target_m < actual)
        {
            return prevOut;
        }

        float err = target - actual;
        int16_t newOut = 0;

        float gainCoef = (abs(err) / (float)(maxRotation));

        if(err > 0.0)
        {
            newOut = prevOut + gain * gainCoef;
        }
        else if(err < 0.0)
        {
            newOut = prevOut - gain * gainCoef;
        }

        prevOut = newOut;

        if(newOut > maxCurrent)
        {
            return maxCurrent;
        }
        else if(newOut < minCurrent)
        {
            return minCurrent;
        }
        else
        {
            return newOut;
        }
    }
}