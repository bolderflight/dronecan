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

/*
  UTC TIME
  RECEIVE TIME
  RETURNING RECEIVE FRAME FROM BUF
  NEED TO LOOK AT EXAMPLE AND FIGURE OUT FUNCTIONS NEEDED
  NEED TO SEPERATE OUT THE MESSAGES






*/






#ifndef SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
#define SRC_UAVCAN_DRIVER_TEENSY_CAN_H_

#include "uavcan/driver/can.hpp"
#include "uavcan/driver/teensy/clock.h"
#include "FlexCAN_T4.h"
#include "circle_buf.h"
#include <array>
#include <algorithm>

namespace uavcan {

// /* Circular buffer size for receiving messages */
// static constexpr std::size_t RX_BUF_SIZE_ = 256;
// /* Circular buffers for receiving CAN messages */
// extern bfs::CircleBuf<CanFrame, RX_BUF_SIZE_> can0_rx_buf_;
// #if defined(__MK66FX1M0__) || defined(__IMXRT1062__)
// extern bfs::CircleBuf<CanFrame, RX_BUF_SIZE_> can1_rx_buf_;
// #endif
// #if defined(__IMXRT1062__)
// extern bfs::CircleBuf<CanFrame, RX_BUF_SIZE_> can2_rx_buf_;
// #endif
// /* CAN interrupt handlers */
// void Can0InterruptHandler(const CAN_message_t &ref);
// #if defined(__MK66FX1M0__) || defined(__IMXRT1062__)
// void Can1InterruptHandler(const CAN_message_t &ref);
// #endif
// #if defined(__IMXRT1062__)
// void Can2InterruptHandler(const CAN_message_t &ref);
// #endif
// /* BFS CAN interface */
// class BfsCanIface : public ICanIface {
//  public:
//   /* Initializes the CAN interface */
//   virtual void begin(const int32_t baud) = 0;
//   /* Returns # of slots available to send messages */
//   virtual int16_t available_to_send() = 0;
//   /* Returns # of slots available to receive messages */
//   virtual int16_t available_to_read() = 0;
// };
// /* CAN interface implementation, templated by CAN bus */
// template<CAN_DEV_TABLE BUS>
// class CanIface : public BfsCanIface {
//  public:
//   ~CanIface() {}
//   /* Initializes the interface */
//   void begin(const int32_t baud) {
//     // can_.begin();
//     // can_.setBaudRate(baud);
//     // can_.setRFFN(RFFN_32);        // 32 FIFO filters
//     // can_.enableFIFO();            // enable FIFO
//     // can_.setMRP(0);               // prioritize FIFO
//     // can_.enableFIFOInterrupt();   // enable the FIFO interrupt
//     // /* Connect FIFO interrupts to interrupt handlers */
//     // #if defined(__IMXRT1062__)
//     // if (BUS == CAN3) {
//     //   can_.onReceive(FIFO, Can2InterruptHandler);
//     // } else if (BUS == CAN2) {
//     //   can_.onReceive(FIFO, Can1InterruptHandler);
//     // } else {
//     //   can_.onReceive(FIFO, Can0InterruptHandler);
//     // }
//     // #elif defined(__MK66FX1M0__)
//     // if (BUS == CAN1) {
//     //   can_.onReceive(FIFO, Can1InterruptHandler);
//     // } else {
//     //   can_.onReceive(FIFO, Can0InterruptHandler);
//     // }
//     // #else
//     // can_.onReceive(FIFO, Can0InterruptHandler);
//     // #endif
//   }
//   /* Returns # of slots available to send messages */
//   int16_t available_to_send() {
//     // return static_cast<uint32_t>(TX_SIZE_) - can_.getTXQueueCount();
//   }
//   /* Returns # of slots available to receive messages */
//   int16_t available_to_read() {
//     // #if defined(__IMXRT1062__)
//     // if (BUS == CAN3) {
//     //   return can2_rx_buf.capacity() - can2_rx_buf.size();
//     // } else if (BUS == CAN2) {
//     //   return can1_rx_buf.capacity() - can1_rx_buf.size();
//     // } else {
//     //   return can0_rx_buf.capacity() - can0_rx_buf.size();
//     // }
//     // #elif defined(__MK66FX1M0__)
//     // if (BUS == CAN1) {
//     //   return can1_rx_buf.capacity() - can1_rx_buf.size();
//     // } else {
//     //   return can0_rx_buf.capacity() - can0_rx_buf.size();
//     // }
//     // #else
//     // return can0_rx_buf.capacity() - can0_rx_buf.size();
//     // #endif
//   }
//   /*
//   * Non-blocking transmission.
//   *
//   * If the frame wasn't transmitted upon TX deadline, the driver should discard
//   * it.
//   *
//   * Note that it is LIKELY that the library will want to send the frames that
//   * were passed into the select() method as the next ones to transmit, but it
//   * is NOT guaranteed. The library can replace those with new frames between
//   * the calls.
//   *
//   * @return 1 = one frame transmitted, 0 = TX buffer full, negative for error.
//   */
//   int16_t send(const CanFrame& frame, MonotonicTime tx_deadline,
//                CanIOFlags flags) {
//     /* Checking the transmit deadline */
//     // if ((sys_clock.getMonotonic() >= tx_deadline) && (!tx_deadline.isZero())) {
//     //   return -1;
//     // }
//     /*
//     * TODO: message might get stored in transmit buffer and not sent before
//     * deadline
//     */
//     // tx_msg_.id = frame.id;
//     // tx_msg_.flags.extended = frame.isExtended();  
//     // tx_msg_.flags.remote = frame.isRemoteTransmissionRequest();
//     // tx_msg_.len = frame.dlc;
//     // for (uint8_t i = 0; i < tx_msg_.len; i++) {
//     //   tx_msg_.buf[i] = frame.data[i];
//     // }
//     // tx_msg_.seq = true;
//     // if (can_.write(tx_msg_) > 0) {
//     //   return 1;
//     // } else {
//     //   return 0;
//     // }
//   }
//   /*
//   * Non-blocking reception.
//   *
//   * Timestamps should be provided by the CAN driver, ideally by the hardware
//   * CAN controller.
//   *
//   * Monotonic timestamp is required and can be not precise since it is needed
//   * only for protocol timing validation (transfer timeouts and inter-transfer
//   * intervals).
//   *
//   * UTC timestamp is optional, if available it will be used for precise time
//   * synchronization; must be set to zero if not available.
//   *
//   * Refer to @ref ISystemClock to learn more about timestamps.
//   *
//   * @param [out] out_ts_monotonic Monotonic timestamp, mandatory.
//   * @param [out] out_ts_utc       UTC timestamp, optional, zero if unknown.
//   * @return 1 = one frame received, 0 = RX buffer empty, negative for error.
//   */
//   int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic,
//                   UtcTime& out_ts_utc, CanIOFlags& out_flags) {
//     // if (can_.read(rx_msg_)) {
//     //   out_ts_monotonic = sys_clock.getMonotonic();
//     //   out_ts_utc = sys_clock.getUtc();
//     //   out_frame.id = rx_msg_.id;
//     //   if (rx_msg_.flags.extended) {
//     //     out_frame.id &= CanFrame::MaskExtID;
//     //     out_frame.id |= CanFrame::FlagEFF;
//     //   }
//     //   out_frame.dlc = rx_msg_.len;
//     //   for (std::size_t i = 0; i < out_frame.dlc; i++) {
//     //     out_frame.data[i] = rx_msg_.buf[i];
//     //   }
//     //   return 1;
//     // }
//     return 0;
//   }
//   /*
//   * Configure the hardware CAN filters. @ref CanFilterConfig.
//   *
//   * @return 0 = success, negative for error.
//   */
//   int16_t configureFilters(const CanFilterConfig* filter_configs,
//                            uint16_t num_configs) {
//     // if (num_configs > NUM_FILTERS_) {return -1;}
//     // can_.setFIFOFilter(REJECT_ALL);
//     // for (uint16_t i = 0; i < num_configs; i++) {
//     //   if (filter_configs[i].id & uavcan::CanFrame::FlagRTR) {
//     //     can_.setFIFOFilter(i, filter_configs[i].id, RTR);
//     //   } else if (filter_configs[i].id & uavcan::CanFrame::FlagEFF) {
//     //     can_.setFIFOFilter(i, filter_configs[i].id, EXT);
//     //   } else {
//     //     can_.setFIFOFilter(i, filter_configs[i].id, STD);
//     //   }
//     // }
//     // return 0;
//   }
//   /*
//   * Number of available hardware filters.
//   */
//   uint16_t getNumFilters() const {return NUM_FILTERS_;}
//   /*
//   * Continuously incrementing counter of hardware errors.
//   * Arbitration lost should not be treated as a hardware error.
//   */
//   uint64_t getErrorCount() const {return error_counter_;}

//  private:
//   // CAN_message_t tx_msg_, rx_msg_;
//   // static constexpr uint16_t NUM_FILTERS_ = 32;
//   // static constexpr FLEXCAN_RXQUEUE_TABLE RX_SIZE_ = RX_SIZE_8;
//   // static constexpr FLEXCAN_TXQUEUE_TABLE TX_SIZE_ = TX_SIZE_256;
//   // FlexCAN_T4<BUS, RX_SIZE_, TX_SIZE_> can_;
//   // uint64_t error_counter_ = 0;
// };
// /* Defining instances of CAN interface depending on number of buses available */
// // #if defined(__IMXRT1062__)
// // CanIface<CAN1> Can1;
// // CanIface<CAN2> Can2;
// // CanIface<CAN3> Can3;
// // #elif defined(__MK66FX1M0__)
// // CanIface<CAN0> Can0;
// // CanIface<CAN1> Can1;
// // #else
// // CanIface<CAN0> Can0;
// // #endif
// /*
// * CAN driver managing the instances, templated by the number of CAN buses to
// * use. Maximum allowed is 3 (MaxCanIfaces) defined in can.hpp.
// */
// template<int8_t NUM_CAN_BUS>
// class CanDriver : public ICanDriver {
//  public:
//   // static_assert(NUM_CAN_BUS <= MaxCanIfaces,
//   //               "More CAN buses than maximum allowed by DroneCAN");
//   // ~CanDriver() {}
//   // CanDriver(const std::array<BfsCanIface *, NUM_CAN_BUS> &ref) : iface_(ref) {}
//   // /* Initializes the CAN interfaces */
//   // void begin(const int32_t baud) {
//   //   for (int8_t i = 0; i < NUM_CAN_BUS; i++) {
//   //     iface_[i]->begin(baud);
//   //   }
//   }
//   /*
//   * Returns an interface by index, or null pointer if the index is out of range.
//   */
//   ICanIface* getIface(uint8_t iface_index) {
//     // if (iface_index > NUM_CAN_BUS) {return UAVCAN_NULLPTR;}
//     // return iface_[iface_index];
//   }
//   /* Returns the number of interfaces */
//   uint8_t getNumIfaces() const {return 0;}
//   /*
//   * Block until the deadline, or one of the specified interfaces becomes
//   * available for read or write.
//   *
//   * Iface masks will be modified by the driver to indicate which exactly
//   * interfaces are available for IO.
//   *
//   * Bit position in the masks defines interface index.
//   *
//   * Note that it is allowed to return from this method even if no requested
//   * events actually happened, or if there are events that were not requested
//   * by the library.
//   *
//   * The pending TX argument contains an array of pointers to CAN frames that
//   * the library wants to transmit next, per interface. This is intended to
//   * allow the driver to properly prioritize transmissions; many drivers will
//   * not need to use it. If a write flag for the given interface is set to one
//   * in the select mask structure, then the corresponding pointer is guaranteed
//   * to be valid (not UAVCAN_NULLPTR).
//   *
//   * @param [in,out] inout_masks        Masks indicating which interfaces are
//   *                                    needed/available for IO.
//   * @param [in]     pending_tx         Array of frames, per interface, that
//   *                                    are likely to be transmitted next.
//   * @param [in]     blocking_deadline  Zero means non-blocking operation.
//   * @return Positive number of ready interfaces or negative error code.
//   */
//   int16_t select(CanSelectMasks& inout_masks,
//                  const CanFrame* (& pending_tx)[MaxCanIfaces],
//                  MonotonicTime blocking_deadline) {
//     // while ((sys_clock.getMonotonic() < blocking_deadline) ||
//     //        (blocking_deadline.isZero())) {
//     //   ready_iface_rx_ = 0;
//     //   ready_iface_tx_ = 0;
//     //   inout_masks.read = 0;
//     //   inout_masks.write = 0;
//     //   for (int8_t i = 0; i < NUM_CAN_BUS; i++) {
//     //     if (iface_[i]->available_to_read() > 0) {
//     //       inout_masks.read |= 1 << i;
//     //       ready_iface_rx_++;
//     //     }
//     //     if (iface_[i]->available_to_send() > 0) {
//     //       inout_masks.write |= 1 << i;
//     //       ready_iface_tx_++;
//     //     }
//     //   }
//     //   ready_iface_ = std::max(ready_iface_rx_, ready_iface_tx_);
//     //   if ((ready_iface_ > 0) || (blocking_deadline.isZero())) {
//     //     return ready_iface_;
//     //   }
//     // }
//     // return -1;
//   }

// //  private:
//   // std::array<BfsCanIface *, NUM_CAN_BUS> iface_;
//   // int16_t ready_iface_rx_;
//   // int16_t ready_iface_tx_;
//   // int16_t ready_iface_;
// };

}  // namespace uavcan

#endif  // SRC_UAVCAN_DRIVER_TEENSY_CAN_H_
