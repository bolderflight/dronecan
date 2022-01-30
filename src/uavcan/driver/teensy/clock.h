/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#ifndef SRC_UAVCAN_DRIVER_TEENSY_CLOCK_H_
#define SRC_UAVCAN_DRIVER_TEENSY_CLOCK_H_

#include "uavcan/driver/system_clock.hpp"

namespace uavcan {

class Clock : public ISystemClock, public Noncopyable {
 public:
  ~Clock() {}
  /*
  * Monototic system clock.
  *
  * This clock shall never jump or change rate; the base time is irrelevant.
  * This clock is mandatory and must remain functional at all times.
  *
  * On POSIX systems use clock_gettime() with CLOCK_MONOTONIC.
  */
  MonotonicTime getMonotonic() const;
  /*
  * Global network clock.
  * It doesn't have to be UTC, the name is a bit misleading - actual time base
  * doesn't matter.
  *
  * This clock can be synchronized with other nodes on the bus, hence it can
  * jump and/or change rate occasionally.
  * This clock is optional; if it is not supported, return zero. Also return
  * zero if the UTC time is not available yet (e.g. the device has just started
  * up with no battery clock).
  *
  * For POSIX refer to clock_gettime(), gettimeofday().
  */
  UtcTime getUtc() const;
  /*
  * Adjust the network-synchronized clock.
  * Refer to @ref getUtc() for details.
  *
  * For POSIX refer to adjtime(), settimeofday().
  *
  * @param [in] adjustment Amount of time to add to the clock value.
  */
  void adjustUtc(UtcDuration adjustment);
  
 private:
  UtcDuration adj_ = UtcDuration::fromUSec(0);
};

extern Clock clock;

}  // namespace uavcan

#endif  // SRC_UAVCAN_DRIVER_TEENSY_CLOCK_H_
