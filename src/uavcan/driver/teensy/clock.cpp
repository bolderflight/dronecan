/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2025 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "uavcan/driver/system_clock.hpp"
#include "uavcan/driver/teensy/clock.h"
#include "Arduino.h"

namespace uavcan {

namespace {
/* 64 bit version of micros() to avoid rollover */
uint64_t micros64() {
  static uint32_t low32, high32;
  uint32_t new_low32 = micros();
  if (new_low32 < low32) {
    high32++;
  }
  low32 = new_low32;
  return((uint64_t) high32 << 32 | low32);
}

}

MonotonicTime Clock::getMonotonic() const {
  return MonotonicTime::fromUSec(micros64());
}

UtcTime Clock::getUtc() const {
  if (!clock_set_) {
    return UtcTime::fromUSec(0);
  } else {
    return UtcTime::fromUSec(micros64() + adj_.toUSec());
  }
}

void Clock::adjustUtc(UtcDuration adjustment) {
  clock_set_ = true;
  adj_ = adjustment;
}

Clock clock;

}  // namespace uavcan
