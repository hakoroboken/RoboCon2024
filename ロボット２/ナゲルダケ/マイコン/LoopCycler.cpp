#include "LoopCycler.h"

namespace hakorobo2024
{
  LoopCycler::LoopCycler(long delta_ms): delta_ms_(delta_ms),prev_time_(0)
  {
  
  }

  void LoopCycler::cycle()
  {
    while(millis() - prev_time_ < delta_ms_)
    {

    }

    prev_time_ = millis();
  }

  float LoopCycler::getDeltaTime()
  {
    return (float)(delta_ms_) / 1000.0;
  }
}