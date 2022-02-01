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
#include "uavcan/transport/can_acceptance_filter_configurator.hpp"
#include "uavcan/equipment/air_data/TrueAirspeed.hpp"

/* Defines needed for restart */
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

/* Node constants */
static constexpr uint32_t NODE_ID = 2;
static constexpr uint8_t SW_VER = 1;
static constexpr uint8_t HW_VER = 1;
static const char* NODE_NAME = "TEST";
static const uint32_t NODE_MEM = 8192;  // size of node memory

/* CAN driver */
uavcan::CanDriver<1> can({&uavcan::Can0});

/* Node and publisher */
uavcan::Node<NODE_MEM> *node;
uavcan::Publisher<uavcan::equipment::air_data::TrueAirspeed> *pub;

/* Data message */
uavcan::equipment::air_data::TrueAirspeed msg;

/* Fake airspeed data */
float airspeed_ms = 0;

/* Function to restart Teensy, inherits from UAVCAN */
class : public uavcan::IRestartRequestHandler {
  bool handleRestartRequest(uavcan::NodeID request_source) override {
    Serial.println("Got a remote restart request!");
    WRITE_RESTART(0x5FA0004);
    return true;
  }
} restart_request_handler;


int main() {
  Serial.begin(115200);
  /* Init CAN transceivers - this is HW config dependent */
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWriteFast(26, LOW);
  digitalWriteFast(27, LOW);
  /* Init can driver */
  can.begin(1000000);
  /* Init Node */
  node = new uavcan::Node<NODE_MEM>(can, uavcan::clock);
  uavcan::protocol::SoftwareVersion sw_ver;
  uavcan::protocol::HardwareVersion hw_ver;
  sw_ver.major = SW_VER;
  sw_ver.minor = 0;
  hw_ver.major = HW_VER;
  hw_ver.minor = 0;
  node->setNodeID(NODE_ID);
  node->setName(NODE_NAME);
  node->setSoftwareVersion(sw_ver);
  node->setHardwareVersion(hw_ver);
  node->setRestartRequestHandler(&restart_request_handler);
  if (node->start() < 0) {
    Serial.println("ERROR starting node");
    while (1) {}
  }
  Serial.println("Node initialized");
  /* Init publisher */
  pub = new uavcan::Publisher<uavcan::equipment::air_data::TrueAirspeed>(*node);
  if (pub->init() < 0) {
    Serial.println("ERROR initializing publisher");
    while (1) {}
  }
  Serial.println("Publisher initialized");
  /* CAN acceptance filters */
  uavcan::configureCanAcceptanceFilters(*node);
  /* Set Node mode to operational */
  node->setModeOperational();
  Serial.println("Setup complete");
  while (1) {
    /* Check the node */
    if (node->spinOnce() < 0) {
      Serial.println("WARNING issue spinning node");
    }
    /* Send the message */
    msg.true_airspeed = airspeed_ms;
    if (pub->broadcast(msg) < 0) {
      Serial.println("WARNING issue publishing message");
    } else {
      Serial.println("Sending message");
    }
    /* Change the airspeed */
    airspeed_ms++;
    if (airspeed_ms > 40.0f) {
      airspeed_ms = 0;
    }
    /* Send at 10 Hz */
    delay(100);
  }
}
