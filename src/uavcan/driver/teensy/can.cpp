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

#include "can.h"

namespace uavcan {

namespace internal {
/* Handles TX interrupts from FlexCAN_T4 */
void TxHandler(const CAN_message_t &msg) {
  uav_can_router.RouteTx(msg);
}
/* UAV CAN router */
UavCanRouter uav_can_router;
}  // internal
/* CAN interfaces */
#if defined(__IMXRT1062__)
CanIface<CAN1> can1;
CanIface<CAN2> can2;
CanIface<CAN3> can3;
#elif defined(__MK66FX1M0__)
CanIface<CAN0> can0;
CanIface<CAN1> can1;
#else
CanIface<CAN0> can0;
#endif
}  // uavcan

