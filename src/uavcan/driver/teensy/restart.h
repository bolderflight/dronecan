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

#ifndef SRC_UAVCAN_DRIVER_TEENSY_RESTART_H_
#define SRC_UAVCAN_DRIVER_TEENSY_RESTART_H_

#include "uavcan/protocol/restart_request_server.hpp"
#include "Arduino.h"

namespace uavcan {

class RestartRequestHandler : public IRestartRequestHandler {
 public:
  bool handleRestartRequest(NodeID request_source) {
    SCB_AIRCR = 0x05FA0004;
    return true;
  }

};

extern RestartRequestHandler restart_request_handler;

}  // namespace uavcan

#endif  // SRC_UAVCAN_DRIVER_TEENSY_RESTART_H_
