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
#include "FlexCAN_T4.h"
#include "circle_buf.h"

namespace uavcan {

/* Utilities needed to route messages from interrupt */
namespace internal {

/* CAN message structure */
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
  /* Initializes the CAN interface */
  virtual bool begin(const int32_t baud) = 0;
  /* Returns # of slots available to send messages */
  virtual int16_t available_to_send() = 0;
  /* Returns # of slots available to receive messages */
  virtual int16_t available_to_read() = 0;

 protected:
  /* OnReceive and OnTransmit interrupt handlers */
  virtual void OnReceive(const CAN_message_t &msg) = 0;
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

  /* Routes received CAN_message_t to the correct UavCanIface derived class */
  void RouteRx(const CAN_message_t &msg) {
    iface_[msg.bus]->OnReceive(msg);
  }

  /* Routes on transmit interrupts to the correct UavCanIface derived class */
  void RouteTx(const CAN_message_t &msg) {
    iface_[msg.bus]->OnTransmit();
  }

 private:
  std::array<UavCanIface *, MAX_NUM_IFACE_> iface_;
} uav_can_router;

/* Handles RX interrupts from FlexCAN_T4 */
void RxHandler(const CAN_message_t &msg) {
  uav_can_router.RouteRx(msg);
}
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
    /* Figure out our bus number */
    #if defined(__IMXRT1062__)
    if (BUS == CAN1) {
      bus_num_ = 1;
      irq_ = IRQ_CAN1;
    } else if (BUS == CAN2) {
      bus_num_ = 2;
      irq_ = IRQ_CAN2;
    } else if (BUS == CAN3) {
      bus_num_ = 3;
      irq_ = IRQ_CAN3;
    }
    #elif defined(__MK20DX256__)
    if (BUS == CAN0) {
      bus_num_ = 0;
      irq_ = IRQ_CAN_MESSAGE;
    }
    #elif defined(__MK64FX512__)
    if (BUS == CAN0) {
      bus_num_ = 0;
      irq_ = IRQ_CAN0_MESSAGE;
    }
    #elif defined(__MK66FX1M0__)
    if (BUS == CAN0) {
      bus_num_ = 0;
      irq_ = IRQ_CAN0_MESSAGE;
    } else if (BUS == CAN1) {
      bus_num_ = 1;
      irq_ = IRQ_CAN1_MESSAGE;
    }
    #endif
  }

  /* Enables using the alternate pins */
  void SetRx(FLEXCAN_PINS pin = DEF) {can_.setRx(pin);}
  void SetTx(FLEXCAN_PINS pin = DEF) {can_.setTx(pin);}

  /* Initializes the interface */
  bool begin(const int32_t baud) {
    can_.begin();
    can_.setBaudRate(baud);
    can_.setRFFN(RFFN_32);        // 32 FIFO filters
    can_.enableFIFO();            // enable FIFO
    can_.setMRP(0);               // prioritize FIFO
    can_.enableFIFOInterrupt();   // enable the FIFO interrupt
    /* Register with router */
    if (!internal::uav_can_router.Register(bus_num_, this)) {
      return false;
    }
    /* Register interrupt handlers */
    can_.onReceive(FIFO, internal::RxHandler);
    can_.onTransmit(internal::TxHandler);
    return true;
  }

  /* Returns # of slots available to send messages */
  int16_t available_to_send() {
    int16_t ret;
    NVIC_DISABLE_IRQ(irq_);
    ret = tx_buf_.capacity() - tx_buf_.size();
    NVIC_ENABLE_IRQ(irq_);
    return ret;
  }

  /* Returns # of slots available to receive messages */
  int16_t available_to_read() {
    int16_t ret;
    NVIC_DISABLE_IRQ(irq_);
    ret = rx_buf_.size();
    NVIC_ENABLE_IRQ(irq_);
    return ret;
  }

  /* Returns true if the TX mailbox is ready to transmit */
  bool tx_ready() {
    return (FLEXCAN_get_code(FLEXCANb_MBn_CS(BUS, TX_MB_NUM_)) ==
            FLEXCAN_MB_CODE_TX_INACTIVE);
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
      can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                 ConvertMessage(tx_msg_));
      return 1;
    } else {
      int16_t ret;
      NVIC_DISABLE_IRQ(irq_);
      ret = tx_buf_.Write(tx_msg_);
      NVIC_ENABLE_IRQ(irq_);
      return ret;
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
    NVIC_DISABLE_IRQ(irq_);
    //   Serial.print("SIZE ");
    // Serial.println(rx_buf_.size());
    bfs::optional<internal::CanMsg> read_val = rx_buf_.Read();

    NVIC_ENABLE_IRQ(irq_);
    
    if (read_val) {
      // Serial.println("READ");
      if (read_val.value().flags.overrun) {
        // Serial.println("OVERRUN");
        error_counter_++;
        return -1;
      } else {
        out_ts_monotonic = read_val.value().monotonic_timestamp;
        out_ts_utc = read_val.value().sync_timestamp;
        out_frame.id = read_val.value().id;
        if (read_val.value().flags.extended) {
          out_frame.id &= uavcan::CanFrame::MaskExtID;
          out_frame.id |= uavcan::CanFrame::FlagEFF;
        }
        if (read_val.value().flags.remote) {
          out_frame.id |= uavcan::CanFrame::FlagRTR;
        }
        out_frame.dlc = read_val.value().len;
        for (int8_t i = 0; i < out_frame.dlc; i++) {
          out_frame.data[i] = read_val.value().buf[i];
        }
        // Serial.println("GOOD");
        return 1;
      }
    } else {
      // Serial.println("NOT READ");
      return 0;
    }
  }
  /*
  * Configure the hardware CAN filters. @ref CanFilterConfig.
  *
  * @return 0 = success, negative for error.
  */
  int16_t configureFilters(const CanFilterConfig* filter_configs,
                           uint16_t num_configs) {
    if (num_configs > NUM_FILTERS_) {return -1;}
    can_.setFIFOFilter(REJECT_ALL);
    for (uint16_t i = 0; i < num_configs; i++) {
      Serial.print("ID\t");
      Serial.print(filter_configs[i].id);
      Serial.print("\tMASK\t");
      Serial.print(filter_configs[i].mask);


      if (filter_configs[i].id & uavcan::CanFrame::FlagEFF) {
        Serial.print("\tEXT\t");
        if (filter_configs[i].id & uavcan::CanFrame::FlagRTR) {
          Serial.print("\tRTR\t");
          if (!can_.setFIFOManualFilter(i, filter_configs[i].id,
                                          filter_configs[i].mask, EXT, RTR)) {
            return -1;
          }
        } else {
          if (!can_.setFIFOManualFilter(i, filter_configs[i].id,
                                          filter_configs[i].mask, EXT)) {
            return -1;
          }
        }
      } else {
        Serial.print("\tSTD\t");
        if (filter_configs[i].id & uavcan::CanFrame::FlagRTR) {
          Serial.print("\tRTR\t");
          if (!can_.setFIFOManualFilter(i, filter_configs[i].id,
                                          filter_configs[i].mask, STD, RTR)) {
            return -1;
          }
        } else {
          if (!can_.setFIFOManualFilter(i, filter_configs[i].id,
                                          filter_configs[i].mask, STD)) {
            return -1;
          }
        }
      }
      Serial.println();
    }
    return 0;
  }
  /*
  * Number of available hardware filters.
  */
  uint16_t getNumFilters() const {return NUM_FILTERS_;}
  /*
  * Continuously incrementing counter of hardware errors.
  * Arbitration lost should not be treated as a hardware error.
  */
  uint64_t getErrorCount() const {return error_counter_;}

 protected:
  /* On receive interrupt handler */
  void OnReceive(const CAN_message_t &msg) {
    if (!rx_buf_.Write(ConvertMessage(msg))) {
      error_counter_++;
    }
  }

  /* On transmit interrupt handler */
  void OnTransmit() {
    /* Check that CAN is ready to send and that we have messages to send */
    if (tx_ready() && tx_buf_.size()) {
      /* Pop a message off the TX buffer */
      tx_buf_.Read(&temp_msg_, 1);
      /* Check the timeout */
      if (clock.getMonotonic() < temp_msg_.monotonic_timestamp) {
        can_.write(static_cast<FLEXCAN_MAILBOX>(TX_MB_NUM_),
                   ConvertMessage(temp_msg_));
      } else {
        /* Timeout reached, still should trigger until the TX buffer is empty */
        if (tx_buf_.size()) {
          OnTransmit();
        }
      }
    }
  }

 private:
  /* Bus number */
  int8_t bus_num_;
  /* TX mailbox number */
  static constexpr int8_t TX_MB_NUM_ = 14;
  /* Maximum number of filters */
  static constexpr int8_t NUM_FILTERS_ = 32;
  /* IQR */
  uint32_t irq_;
  /* Temp message */
  internal::CanMsg temp_msg_, tx_msg_;
  /* CAN driver */
  FlexCAN_T4<BUS, RX_SIZE_2, TX_SIZE_2> can_;
  /* Buffer sizes */
  static constexpr std::size_t BUF_SIZE_ = 256;
  /* Circular buffers to hold RX and TX messages */
  bfs::CircleBuf<internal::CanMsg, BUF_SIZE_> rx_buf_;
  bfs::CircleBuf<internal::CanMsg, BUF_SIZE_> tx_buf_;
  /* Error count */
  uint64_t error_counter_ = 0;
  /* Convert from CAN_message_t to CanMsg */
  internal::CanMsg ConvertMessage(const CAN_message_t &msg) {
    internal::CanMsg ret;
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
  CAN_message_t ConvertMessage(const internal::CanMsg &msg) {
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
/* Defining instances of CAN interface depending on number of buses available */
#if defined(__IMXRT1062__)
CanIface<CAN1> Can1;
CanIface<CAN2> Can2;
CanIface<CAN3> Can3;
#elif defined(__MK66FX1M0__)
CanIface<CAN0> Can0;
CanIface<CAN1> Can1;
#else
CanIface<CAN0> Can0;
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
  /* Initializes the CAN interfaces */
  bool begin(const int32_t baud) {
    for (int8_t i = 0; i < NUM_CAN_BUS; i++) {
      if (!iface_[i]->begin(baud)) {
        return false;
      }
    }
    return true;
  }
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
