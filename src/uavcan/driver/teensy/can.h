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

#ifndef SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
#define SRC_UAVCAN_DRIVER_TEENSY_CAN_H_

#include <array>
#include "uavcan/driver/can.hpp"
#include "uavcan/driver/teensy/clock.h"
#include "flexcan.h"
#include "circle_buf.h"

namespace uavcan {

/* Utilities needed to route messages from interrupt */
namespace internal {

/*
* Maximum number of buses, really it's 3, but on Teensy 4.x, the buses are
* numbered CAN1, CAN2, CAN3 instead of base 0.
*/
static constexpr int8_t MAX_NUM_IFACE_ = 4;
static constexpr int8_t MAX_IFACE_INDEX_ = MAX_NUM_IFACE_ - 1;

/* Interface for CAN implementation */
class UavCanIface : public ICanIface {
  friend class UavCanRouter;
 public:
  /* Returns true if available to send a message */
  virtual bool available_to_send() = 0;
  /* Returns true if message sitting in read buffer */
  virtual bool available_to_read() = 0;

 protected:
  /* OnTransmit interrupt handlers */
  virtual void OnTransmit() = 0;
};

/*
* Manages an address book of UavCanIface derived classes based on bus number
* and routes messages to the correct one.
*/
class UavCanRouter {
 public:
  /* Registers an UavCanIface derived class with the router by bus number */
  bool Register(const int8_t bus_num, UavCanIface * can) {
    if ((bus_num < 0) || (bus_num > MAX_IFACE_INDEX_)) {return false;}
    iface_[bus_num] = can;
    return true;
  }

  /* Routes on transmit interrupts to the correct UavCanIface derived class */
  void RouteTx(const CAN_message_t &msg) {
    iface_[msg.bus]->OnTransmit();
  }

 private:
  std::array<UavCanIface *, MAX_NUM_IFACE_> iface_;
} uav_can_router;

/* Handles TX interrupts from FlexCAN_T4 */
void TxHandler(const CAN_message_t &msg) {
  uav_can_router.RouteTx(msg);
}

}  // namespace internal

/* CAN interface implementation, templated by CAN bus and buffer sizes */
template<CAN_DEV_TABLE BUS>
class CanIface : public internal::UavCanIface {
 public:
  CanIface() {
    /* Figuring out the bus number */
    if (BUS == CAN0) {
      bus_num_ = 0;
    } else if (BUS == CAN1) {
      bus_num_ = 1;
    } else if (BUS == CAN2) {
      bus_num_ = 2;
    } else if (BUS == CAN3) {
      bus_num_ = 3;
    }
  }

  /* Initialize CAN bus */
  void begin() {
    /* Initialize CAN */
    can_.begin();
    /* Setup mailboxes */
    can_.setMaxMB(MAX_NUM_MB_);
    for (uint8_t i = 0; i < MAX_NUM_RX_MB_; i++) {
      can_.setMB(static_cast<FLEXCAN_MAILBOX>(i), RX, EXT);
    }
    can_.setMB(static_cast<FLEXCAN_MAILBOX>(TX_MB_), TX);
    /* Register with router */
    internal::uav_can_router.Register(bus_num_, this);
    /* Register interrupts */
    can_.onTransmit(internal::TxHandler);
  }

  /* Set alternative TX and RX pins */
  void setTX(FLEXCAN_PINS pin = DEF) {can_.setTX(pin);}
  void setRX(FLEXCAN_PINS pin = DEF) {can_.setRX(pin);}

  /* Set the baudrate */
  void setBaudRate(const int32_t baud) {can_.setBaudRate(baud);}

  /* Returns true if the TX mailbox is ready to transmit */
  bool tx_ready() {
    return (FLEXCAN_get_code(FLEXCANb_MBn_CS(BUS, TX_MB_)) ==
            FLEXCAN_MB_CODE_TX_INACTIVE);
  }

  /* Returns slots available in send message buffer */
  bool available_to_send() {
    return (tx_buf_.capacity() - tx_buf_.size()) > 0;
  }

  /* Returns number of message available to read */
  bool available_to_read() {
    bool ret = false;
    if (can_.read(temp_msg_) > 0) {
      ret = true;
      rx_msg_ = ConvertMessage(temp_msg_);
    }
    return ret;
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
    if (clock.getMonotonic() >= tx_deadline) {
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
    tx_msg_.monotonic_timestamp = tx_deadline;
    /* Check to see if we can transmit immediately */
    if (tx_ready()) {
      can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_),
                 ConvertMessage(tx_msg_));
      return 1;
    } else {
      return tx_buf_.Write(tx_msg_);
    }
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
    out_ts_monotonic = rx_msg_.monotonic_timestamp;
    out_ts_utc = rx_msg_.sync_timestamp;
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
        can_.setMBManualFilter(static_cast<FLEXCAN_MAILBOX>(i),
                              filter_configs[i].id,
                              filter_configs[i].mask);
      } else {
        can_.setMB(static_cast<FLEXCAN_MAILBOX>(i), RX, STD);
        can_.setMBManualFilter(static_cast<FLEXCAN_MAILBOX>(i),
                              filter_configs[i].id,
                              filter_configs[i].mask);
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

 protected:

  /* On transmit interrupt handler */
  void OnTransmit() {
    /* Check that CAN is ready to send and that we have messages to send */
    if (tx_ready() && tx_buf_.size()) {
      /* Pop a message off the TX buffer */
      tx_buf_.Read(&tx_msg_, 1);
      /* Check the timeout */
      if (clock.getMonotonic() < tx_msg_.monotonic_timestamp) {
        can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_),
                   ConvertMessage(tx_msg_));
      } else {
        /* Timeout reached, still should trigger until the TX buffer is empty */
        if (tx_buf_.size()) {
          OnTransmit();
        }
      }
    }
  }


 private:
  /* CAN message */
  struct CanMsg {
    struct {
      bool extended = 0;
      bool remote = 0;
      bool overrun = 0;
    } flags;
    uint8_t len = 8;
    uint8_t buf[8] = {0};
    uint32_t id;
    MonotonicTime monotonic_timestamp;
    UtcTime sync_timestamp;
  };
  /* CAN driver */
  FlexCAN_T4<BUS, RX_SIZE_2, TX_SIZE_2> can_;
  /* Buffer sizes */
  static constexpr std::size_t BUF_SIZE_ = 256;
  /* Circular buffers to hold RX and TX messages */
  bfs::CircleBuf<CanMsg, BUF_SIZE_> tx_buf_;
  /* Bus number */
  int8_t bus_num_;
  /* Maximum number of mailboxes */
  #if defined(__IMXRT1062__)
  static constexpr uint8_t MAX_NUM_MB_ = 64;
  #else
  static constexpr uint8_t MAX_NUM_MB_ = 16;
  #endif
  /* Maximum number of receive mailboxes */
  static constexpr uint8_t MAX_NUM_RX_MB_ = MAX_NUM_MB_ - 1;
  /* TX mailbox number */
  static constexpr uint8_t TX_MB_ = MAX_NUM_MB_ - 1;
  /* Temp messages */
  CanMsg tx_msg_, rx_msg_;
  CAN_message_t temp_msg_;
  /* Error count */
  uint64_t error_counter_ = 0;
  /* Convert from CAN_message_t to CanMsg */
  CanMsg ConvertMessage(const CAN_message_t &msg) {
    CanMsg ret;
    ret.monotonic_timestamp = clock.getMonotonic();
    ret.sync_timestamp = clock.getUtc();
    ret.id = msg.id;
    ret.flags.extended = msg.flags.extended;
    ret.flags.remote = msg.flags.remote;
    ret.flags.overrun = msg.flags.overrun;
    ret.len = msg.len;
    for (std::size_t i = 0; i < ret.len; i++) {
      ret.buf[i] = msg.buf[i];
    }
    return ret;
  }
  /* Convert from CanMsg to CAN_message_t */
  CAN_message_t ConvertMessage(const CanMsg &msg) {
    CAN_message_t ret;
    ret.id = msg.id;
    ret.flags.extended = msg.flags.extended;
    ret.flags.remote = msg.flags.remote;
    ret.flags.overrun = msg.flags.overrun;
    ret.len = msg.len;
    for (std::size_t i = 0; i < ret.len; i++) {
      ret.buf[i] = msg.buf[i];
    }
    return ret;
  }
};
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

/*
* CAN driver managing the instances, templated by the number of CAN buses to
* use. Maximum allowed is 3 (MaxCanIfaces) defined in can.hpp.
*/
template<int8_t NUM_CAN_BUS>
class CanDriver : public ICanDriver {
 public:
  static_assert(NUM_CAN_BUS <= MaxCanIfaces,
                "More CAN buses than maximum allowed by DroneCAN");
  CanDriver(const std::array<internal::UavCanIface *, NUM_CAN_BUS> &ref) :
            iface_(ref) {}
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
    while ((clock.getMonotonic() < blocking_deadline) ||
           (blocking_deadline.isZero())) {
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
  std::array<internal::UavCanIface *, NUM_CAN_BUS> iface_;
  int16_t ready_iface_rx_;
  int16_t ready_iface_tx_;
  int16_t ready_iface_;
};

}  // namespace uavcan

#endif  // SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
