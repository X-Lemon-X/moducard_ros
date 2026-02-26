/*
Copyright (c) 2025 Patryk Dudziński

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Authors: Patryk Dudziński
 */

#pragma once

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "mc_plugin_base/mc_firmware/can_base.hpp"
#include "mc_plugin_base/mc_firmware/mc_common.hpp"
#include "mc_plugin_base/mc_firmware/status.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

namespace mcan {

template<typename T>
concept CanFrameType = std::same_as<T, can_frame> || std::same_as<T, canfd_frame>;

/// @brief Convert the CanFrame to a SocketCAN can_frame structure.
template<CanFrameType T>
T
to_can_frame(const CanFrame& can_frame)
{
  T frame;
  frame.can_id = can_frame.id | (can_frame.is_extended ? CAN_EFF_FLAG : 0) |
                 (can_frame.is_remote_request ? CAN_RTR_FLAG : 0);
  frame.len = can_frame.size;
  memcpy(frame.data, can_frame.data, can_frame.size);
  return frame;
};

/// @brief Convert a SocketCAN can_frame structure to a CanFrame.
template<CanFrameType T>
CanFrame
from_can_frame(const T& frame)
{
  CanFrame cf;
  cf.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  cf.is_remote_request = (frame.can_id & CAN_RTR_FLAG) != 0;
  cf.size = frame.len;
  cf.id = frame.can_id & CAN_ERR_MASK;
  memcpy(cf.data, frame.data, frame.len);
  return cf;
};
// };

class CanDriver : public CanBase
{
 public:
  virtual ~CanDriver();

  /// @brief Create a new CanDriver instance.
  /// @param can_interface Name of the CAN interface to use (e.g. "can0").
  /// @param threaded If true, the driver creates two thread to run callback and handel
  /// can communication.
  /// @param timeout_us Timeout for the CAN socket in microseconds should be big if thread
  /// is used.
  /// @param queue_size Size of the queue for sending CAN frames.
  /// @return A shared pointer to the CanDriver instance.
  /// @note The queue size should be big enough to handle the number of frames that will
  /// be sent.
  static Result<std::shared_ptr<CanDriver>> Make(const std::string& can_interface,
                                                 uint32_t timeout_us = 100000,
                                                 size_t queue_size = 32,
                                                 size_t buffer_size = 512);

  /// @brief Send a CAN frame to the CAN bus.
  /// @param frame The CAN frame to send.
  /// @note The frame will be sent to the CAN bus immediately if the driver is not
  /// threaded.
  /// @return Status of the operation.
  virtual Status send(const CanFrame& frame) override;

  virtual Result<CanFrame> send_await_response(const CanFrame& frame,
                                               uint32_t response_id,
                                               uint32_t timeout_ms = 1000) override;

  /// @brief Add a callback for a specific CAN ID.
  /// @param id The CAN ID to listen for, if you want to receive callback for a remote
  /// request you have to set "id = SOME_CAN_ID | CAN_RTR_FLAG".
  /// @param callback The callback function to call when a frame with the specified ID is
  /// received.
  /// @param args Optional arguments to pass to the callback function.
  /// @return Status of the operation.
  /// @note Callback can only be added before opening the CAN socket
  /// @note When no callback are registered, and the thread is used, the driver will not
  /// receive any frames.
  /// @note When normal mode is used (instead of threaded), the driver don't have to have
  /// any callback registered. but then handling incoming data will have to be implemented
  /// by the user after receiving the frame.
  virtual Status add_callback(uint32_t id,
                              can_callback_type callback,
                              void* args = nullptr) override;

  /// @brief Add a callback for a specific CAN IDs with mask.
  /// so one callback can be used for multiple CAN IDs by using mask to specify which bits
  /// of the ID should be matched.
  /// @param id_base The base CAN ID to listen for, the bits that are not
  /// masked will be ignored when matching incoming frames.
  /// @param id_mask The mask to apply to incoming CAN IDs when matching, bits set
  /// to 1 in the mask will be compared to the corresponding bits in the base ID, bits set
  /// to 0 will be ignored.
  /// @param callback The callback function to call when a frame with the specified ID is
  /// received.
  /// @param args Optional arguments to pass to the callback function.
  /// @return Status of the operation.
  /// @note When using masked callbacks, if a id matches a masked callback and a
  /// non-masked callback, only the regural callback will be called, and the masked one
  /// will be ignored. if matching multiple masked callbacks, only the first one that
  /// matches will be called, the order of matching is not guaranteed, so if you have
  /// multiple masked callbacks that can match the same ID, you should make sure that the
  /// most specific one (the one with the most bits set in the mask) is added first, to
  /// ensure that it will be matched before less specific ones. will be called.
  virtual Status add_callback_masked(uint32_t id_base,
                                     uint32_t id_mask,
                                     can_callback_type callback,
                                     void* args = nullptr) override;

  /// @brief Remove a callback for a specific CAN ID.
  /// @param id The CAN ID to remove the callback for.
  /// @return Status of the operation.
  /// @note Callback can only be removed before opening the CAN socket
  virtual Status remove_callback(uint32_t id) override;

  /// @brief Remove a masked callback for a specific CAN ID.
  /// @param id_base The base CAN ID of the callback to remove.
  /// @param id_mask The mask of the callback to remove.
  /// @return Status of the operation.
  virtual Status remove_callback_masked(uint32_t id_base, uint32_t id_mask) override;

  /// @brief Open the CAN socket.
  /// @note This function will create a new thread to handle the CAN socket.
  /// @return Status of the operation.
  virtual Status open_can() override;

  /// @brief Close the CAN socket if is the last one.
  /// @param can_driver The CanDriver instance to close.
  /// @note This function will reset the pointer to the CanDriver instance, when its the
  /// last one then CAN will be closed.
  /// @note This function will not close the CAN socket if there are other instances using
  /// it.
  /// @return Status of the operation.
  virtual Status close_can() override;

 private:
  CanDriver(const std::string& can_interface,
            uint32_t timeout_us,
            size_t queue_size = 32,
            size_t buffer_size = 512);

  /// @brief Handle CAN transmission in a separate thread.
  void handle_can_transmission();

  /// @brief Handle CAN tasks in a separate thread.
  void handle_can_tasks();

  /// @brief Send a CAN frame to the queue. that will be sent to the CAN bus.
  Status send_to_queue(const CanFrame& frame);

  /// @brief Write a CAN frame to the socket.
  Status write_can_frame(const CanFrame& frame);

  /// @brief Read a CAN frame from the socket.
  Result<CanFrame> read_can_frame();

  Result<CanFrame> read_and_handle_frame();

  Status internall_close_can();

  struct PendingResponse
  {
    CanFrame frame = {};
    std::condition_variable cv;
    std::mutex mtx;
    bool received = false;
  };

  std::queue<CanFrame> send_queue;
  std::queue<CanFrame> receive_queue;
  std::condition_variable queue_condition;
  std::mutex queue_mutex;
  std::mutex can_mutex;
  std::mutex interface_mutex;
  std::unordered_map<uint32_t, std::pair<can_callback_type, void*>> callbacks_;

  std::vector<
    std::pair<std::pair<uint32_t, uint32_t>, std::pair<can_callback_type, void*>>>
    masked_callbacks_;

  std::thread can_write_thread;
  std::thread can_perform_tasks_thread;
  std::atomic<bool> run_thread_write{ false };
  std::atomic<bool> run_thread_perform_tasks{ false };

  std::string can_interface_name;
  size_t queue_size;
  int socket_fd;
  size_t buffer_size;
  bool is_open = false;

  const uint32_t _timeout_us;
  sockaddr_can _addr;
  ifreq _ifr;

  int _close_calls = 0;

  static std::vector<std::shared_ptr<CanBase>> _global_can_driver;

  std::unordered_map<uint32_t, PendingResponse*> _pending_responses;
};

}; // namespace mcan