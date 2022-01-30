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

#include "uavcan.h"
#include "uavcan/equipment/ahrs/RawIMU.hpp"

// uavcan::Clock clock;
// uavcan::CanDriver<1> pub_driver({&uavcan::Can0});
// uavcan::CanDriver<1> sub_driver({&uavcan::Can1});
// static const unsigned int NODE_MEMORY_POOL = 8192;
// uavcan::Node<NODE_MEMORY_POOL> pub_node(pub_driver, clock);
// uavcan::Publisher<uavcan::equipment::ahrs::RawIMU> pub(pub_node);
// uavcan::Node<NODE_MEMORY_POOL> sub_node(sub_driver, clock);
// uavcan::Subscriber<uavcan::equipment::ahrs::RawIMU> sub(sub_node);

void imu_data_callback(const uavcan::equipment::ahrs::RawIMU& msg) {
  // Serial.println("RX");
}

int main() {
  uavcan::clock.getMonotonic();
  // /* Serial to print info */
  // Serial.begin(115200);
  // while (!Serial) {}
  // Serial.println("TEST BEGIN");
  // /* Init the CAN modules */
  // pinMode(26, OUTPUT);
  // pinMode(27, OUTPUT);
  // digitalWriteFast(26, LOW);
  // digitalWriteFast(27, LOW);

  // pub_driver.Begin(500000);
  // sub_driver.Begin(500000);
  // sub.start(imu_data_callback);
  // if (pub.init() < 0) {
  //   Serial.println("ERROR");
  // }
  // uavcan::equipment::ahrs::RawIMU tx_msg;
  // tx_msg.timestamp = clock.getUtc();
  // while (1) {
  //   pub.broadcast(tx_msg);
  //   delay(100);
  // }
}

