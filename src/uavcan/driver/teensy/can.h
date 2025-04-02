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

#ifndef SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
#define SRC_UAVCAN_DRIVER_TEENSY_CAN_H_

#include <array>
#include "uavcan/driver/can.hpp"
#include "uavcan/driver/teensy/clock.h"
#include "FlexCAN_T4.h"

namespace uavcan {

class UavCanIface : public ICanIface {
 public:
  /* Returns true if available to send a message */
  virtual bool available_to_send() = 0;
  /* Returns true if message sitting in read buffer */
  virtual bool available_to_read() = 0;
};

/* CAN interface implementation, templated by CAN bus and buffer sizes */
template<CAN_DEV_TABLE BUS>
class CanIface : public UavCanIface {
 public:
  CanIface() {}
  /* Initialize CAN bus */
  void begin() {
    /* Initialize CAN */
    can_.begin();
    can_.setMaxMB(MAX_NUM_MB_);
    for (uint8_t i = 0; i < MAX_NUM_RX_MB_; i++) {
      can_.setMB(static_cast<FLEXCAN_MAILBOX>(i), RX, EXT);
    }
    can_.setMB(static_cast<FLEXCAN_MAILBOX>(TX_MB_), TX);
  }
  /* Set alternative TX and RX pins */
  void setTX(FLEXCAN_PINS pin = DEF) {can_.setTX(pin);}
  void setRX(FLEXCAN_PINS pin = DEF) {can_.setRX(pin);}
  /* Set the baudrate */
  void setBaudRate(const int32_t baud) {can_.setBaudRate(baud);}
  /* Returns true if slots available in send message buffer */
  bool available_to_send() {
    return (can_.getTXQueueCount() < static_cast<uint16_t>(TX_SIZE_64));
  }
  /* Returns true if message available to read */
  bool available_to_read() {
    return can_.read(rx_msg_);
  }
  /*
  * Non-blocking transmission.
  *
  * If the frame wasn't transmitted upon TX deadline, the driver should discard
  * it.
  *
  * Note that it is LIKELY that the library will want to send the frames that
  * were passed into the select() method as the next ones to transmit, but it
  * is NOT guaranteed. The library can replace those with new frames between
  * the calls.
  *
  * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
  */
  int16_t send(const CanFrame& frame, MonotonicTime tx_deadline,
               CanIOFlags flags) {
    /* Checking the transmit deadline */
    if ((!tx_deadline.isZero()) && (clock.getMonotonic() >= tx_deadline)) {
      return -1;
    }
    /* Build the message */
    tx_msg_.id = frame.id;
    tx_msg_.flags.extended = frame.isExtended();
    tx_msg_.flags.remote = frame.isRemoteTransmissionRequest();
    tx_msg_.len = frame.dlc;
    for (int8_t i = 0; i < tx_msg_.len; i++) {
      tx_msg_.buf[i] = frame.data[i];
    }
    int16_t ret = can_.write(tx_msg_);
    return ret;
  }
  /*
  * Non-blocking reception.
  *
  * Timestamps should be provided by the CAN driver, ideally by the hardware
  * CAN controller.
  *
  * Monotonic timestamp is required and can be not precise since it is needed
  * only for protocol timing validation (transfer timeouts and inter-transfer
  * intervals).
  *
  * UTC timestamp is optional, if available it will be used for precise time
  * synchronization; must be set to zero if not available.
  *
  * Refer to @ref ISystemClock to learn more about timestamps.
  *
  * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
  * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
  * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
  */
  int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic,
                  UtcTime& out_ts_utc, CanIOFlags& out_flags) {
    out_ts_monotonic = clock.getMonotonic();;
    out_ts_utc = clock.getUtc();
    out_frame.id = rx_msg_.id;
    if (rx_msg_.flags.extended) {
      out_frame.id &= uavcan::CanFrame::MaskExtID;
      out_frame.id |= uavcan::CanFrame::FlagEFF;
    }
    if (rx_msg_.flags.remote) {
      out_frame.id |= uavcan::CanFrame::FlagRTR;
    }
    out_frame.dlc = rx_msg_.len;
    for (int8_t i = 0; i < out_frame.dlc; i++) {
      out_frame.data[i] = rx_msg_.buf[i];
    }
    return 1;
  }

  /*
  * Configure the hardware CAN filters. @ref CanFilterConfig.
  *
  * @return 0 = success, negative for error.
  */
  int16_t configureFilters(const CanFilterConfig* filter_configs,
                           uint16_t num_configs) {
    if (num_configs > MAX_NUM_RX_MB_) {return -1;}
    can_.setMBFilter(REJECT_ALL);
    for (uint16_t i = 0; i < num_configs; i++) {
      if (filter_configs[i].id & uavcan::CanFrame::FlagEFF) {
        can_.setMB(static_cast<FLEXCAN_MAILBOX>(i), RX, EXT);
        can_.setMBManualFilter(static_cast<FLEXCAN_MAILBOX>(i), filter_configs[i].id, filter_configs[i].mask);
      } else {
        can_.setMB(static_cast<FLEXCAN_MAILBOX>(i), RX, STD);
        can_.setMBManualFilter(static_cast<FLEXCAN_MAILBOX>(i), filter_configs[i].id, filter_configs[i].mask);
      }
    }
    return 0;
  }
  /*
  * Number of available hardware filters.
  */
  uint16_t getNumFilters() const {return MAX_NUM_RX_MB_;}
  /*
  * Continuously incrementing counter of hardware errors.
  * Arbitration lost should not be treated as a hardware error.
  */
  uint64_t getErrorCount() const {return error_counter_;}

 private:
  /* CAN driver */
  FlexCAN_T4<BUS, RX_SIZE_16, TX_SIZE_64> can_;
  /* Flag for message available */
  bool msg_avail_ = false;
  /* Maximum number of total mailboxes */
  #if defined(__IMXRT1062__)
  static constexpr uint8_t MAX_NUM_MB_ = 64;
  #else
  static constexpr uint8_t MAX_NUM_MB_ = 16;
  #endif
  /* Maximum number of receive mailboxes */
  static constexpr uint8_t MAX_NUM_RX_MB_ = MAX_NUM_MB_ - 1;
  /* TX mailbox number */
  static constexpr uint8_t TX_MB_ = MAX_NUM_MB_ - 1;
  /* Error count */
  uint64_t error_counter_ = 0;
  /* Temporary messages */
  CAN_message_t tx_msg_, rx_msg_;
};

/*
* CAN driver managing the instances, templated by the number of CAN buses to
* use. Maximum allowed is 3 (MaxCanIfaces) defined in can.hpp.
*/
template<int8_t NUM_CAN_BUS>
class CanDriver : public ICanDriver {
 public:
  static_assert(NUM_CAN_BUS <= MaxCanIfaces,
                "More CAN buses than maximum allowed by DroneCAN");
  CanDriver(const std::array<UavCanIface *, NUM_CAN_BUS> &ref) : iface_(ref) {}
  /*
  * Returns an interface by index, or null pointer if the index is out of range.
  */
  ICanIface* getIface(uint8_t iface_index) {
    if (iface_index > NUM_CAN_BUS) {return UAVCAN_NULLPTR;}
    return iface_[iface_index];
  }
  /* Returns the number of interfaces */
  uint8_t getNumIfaces() const {return NUM_CAN_BUS;}
  /*
  * Block until the deadline, or one of the specified interfaces becomes
  * available for read or write.
  *
  * Iface masks will be modified by the driver to indicate which exactly
  * interfaces are available for IO.
  *
  * Bit position in the masks defines interface index.
  *
  * Note that it is allowed to return from this method even if no requested
  * events actually happened, or if there are events that were not requested
  * by the library.
  *
  * The pending TX argument contains an array of pointers to CAN frames that
  * the library wants to transmit next, per interface. This is intended to
  * allow the driver to properly prioritize transmissions; many drivers will
  * not need to use it. If a write flag for the given interface is set to one
  * in the select mask structure, then the corresponding pointer is guaranteed
  * to be valid (not UAVCAN_NULLPTR).
  *
  * @param [in,out] inout_masks        Masks indicating which interfaces are
  *                                    needed/available for IO.
  * @param [in]     pending_tx         Array of frames, per interface, that
  *                                    are likely to be transmitted next.
  * @param [in]     blocking_deadline  Zero means non-blocking operation.
  * @return Positive number of ready interfaces or negative error code.
  */
  int16_t select(CanSelectMasks& inout_masks,
                 const CanFrame* (& pending_tx)[MaxCanIfaces],
                 MonotonicTime blocking_deadline) {
    while ((clock.getMonotonic() < blocking_deadline) || (blocking_deadline.isZero())) {
      ready_iface_rx_ = 0;
      ready_iface_tx_ = 0;
      inout_masks.read = 0;
      inout_masks.write = 0;
      for (int8_t i = 0; i < NUM_CAN_BUS; i++) {
        if (iface_[i]->available_to_read() > 0) {
          inout_masks.read |= 1 << i;
          ready_iface_rx_++;
        }
        if (iface_[i]->available_to_send() > 0) {
          inout_masks.write |= 1 << i;
          ready_iface_tx_++;
        }
      }
      ready_iface_ = std::max(ready_iface_rx_, ready_iface_tx_);
      if ((ready_iface_ > 0) || (blocking_deadline.isZero())) {
        return ready_iface_;
      }
    }
    return -1;
  }

 private:
  std::array<UavCanIface *, NUM_CAN_BUS> iface_;
  int16_t ready_iface_rx_;
  int16_t ready_iface_tx_;
  int16_t ready_iface_;
};

}  // namespace uavcan

#endif  // SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
