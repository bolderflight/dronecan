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

/* Base functionality */
#include "uavcan/uavcan.hpp"
#include "uavcan/driver/teensy/clock.h"
#include "uavcan/driver/teensy/can.h"

/* Common messages */
#include "uavcan/equipment/actuator/ArrayCommand.hpp"
#include "uavcan/equipment/actuator/Command.hpp"
#include "uavcan/equipment/actuator/Status.hpp"
#include "uavcan/equipment/ahrs/MagneticFieldStrength.hpp"
#include "uavcan/equipment/ahrs/MagneticFieldStrength2.hpp"
#include "uavcan/equipment/ahrs/RawIMU.hpp"
#include "uavcan/equipment/ahrs/Solution.hpp"
#include "uavcan/equipment/air_data/AngleOfAttack.hpp"
#include "uavcan/equipment/air_data/IndicatedAirspeed.hpp"
#include "uavcan/equipment/air_data/RawAirData.hpp"
#include "uavcan/equipment/air_data/Sideslip.hpp"
#include "uavcan/equipment/air_data/StaticPressure.hpp"
#include "uavcan/equipment/air_data/StaticTemperature.hpp"
#include "uavcan/equipment/air_data/TrueAirspeed.hpp"
#include "uavcan/equipment/camera_gimbal/AngularCommand.hpp"
#include "uavcan/equipment/camera_gimbal/GEOPOICommand.hpp"
#include "uavcan/equipment/camera_gimbal/Mode.hpp"
#include "uavcan/equipment/camera_gimbal/Status.hpp"
#include "uavcan/equipment/device/Temperature.hpp"
#include "uavcan/equipment/esc/RawCommand.hpp"
#include "uavcan/equipment/esc/RPMCommand.hpp"
#include "uavcan/equipment/esc/Status.hpp"
#include "uavcan/equipment/gnss/Auxiliary.hpp"
#include "uavcan/equipment/gnss/ECEFPositionVelocity.hpp"
#include "uavcan/equipment/gnss/Fix.hpp"
#include "uavcan/equipment/gnss/Fix2.hpp"
#include "uavcan/equipment/gnss/RTCMStream.hpp"
#include "uavcan/equipment/hardpoint/Command.hpp"
#include "uavcan/equipment/hardpoint/Status.hpp"
#include "uavcan/equipment/ice/reciprocating/CylinderStatus.hpp"
#include "uavcan/equipment/ice/reciprocating/Status.hpp"
#include "uavcan/equipment/ice/FuelTankStatus.hpp"
#include "uavcan/equipment/indication/BeepCommand.hpp"
#include "uavcan/equipment/indication/LightsCommand.hpp"
#include "uavcan/equipment/indication/RGB565.hpp"
#include "uavcan/equipment/indication/SingleLightCommand.hpp"
#include "uavcan/equipment/power/BatteryInfo.hpp"
#include "uavcan/equipment/power/CircuitStatus.hpp"
#include "uavcan/equipment/power/PrimaryPowerSupplyStatus.hpp"
#include "uavcan/equipment/range_sensor/Measurement.hpp"
#include "uavcan/equipment/safety/ArmingStatus.hpp"
#include "uavcan/navigation/GlobalNavigationSolution.hpp"
