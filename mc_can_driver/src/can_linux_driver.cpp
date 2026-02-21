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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <cstring>

#include <unistd.h>

#include <string>

#include "iostream"

#include "mc_plugin_base/status.hpp"
#include "mc_can_driver/can_linux_driver.hpp"


using namespace mcan;


CanDriver::CanDriver(const std::string &can_interface, uint32_t _timeout_us, size_t _queue_size, size_t _buffer_size)
: CanBase(), can_interface_name(can_interface), queue_size(_queue_size), socket_fd(-1),
  buffer_size(_buffer_size), is_open(false), _timeout_us(_timeout_us), _addr({}), _ifr({}) {
}

CanDriver::~CanDriver() {
  internall_close_can();
}

Result<std::shared_ptr<CanDriver>>
CanDriver::Make(const std::string &can_interface, uint32_t timeout_us, size_t _queue_size, size_t _buffer_size) {
  if(can_interface.empty()) {
    return Status::Invalid("CAN interface is empty");
  }
  if(timeout_us <= 0) {
    return Status::Invalid("Invalid timeout value");
  }

  if(_queue_size <= 0) {
    return Status::Invalid("Invalid queue size");
  }

  if(_buffer_size <= CANFD_MAX_DLEN) {
    return Status::Invalid("Invalid buffer size");
  }

  return Result<std::shared_ptr<CanDriver>>::OK(
  std::shared_ptr<CanDriver>(new CanDriver(can_interface, timeout_us, _queue_size, _buffer_size)));
}

Status CanDriver::open_can() {
  if(is_open) {
    return Status::OK();
    // return Status::Invalid("CAN socket is already open");
  }

  // check if we have registered callbacks
  // if(callbacks_.empty()) {
  //   return Status::Invalid("No can callbacks registered");
  // }

  // Open the CAN socket
  socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(socket_fd < 0) {
    return Status::ExecutionError("CanDriver: Failed to open CAN socket");
  }

  // Set the socket to non-blocking mode
  strncpy(_ifr.ifr_name, can_interface_name.c_str(), can_interface_name.size());
  if(ioctl(socket_fd, SIOCGIFINDEX, &_ifr) < 0) {
    return Status::ExecutionError("CanDriver: Failed to get CAN interface index name:" + can_interface_name);
  }

  // creat filter for all the registered callbacks
  std::vector<can_filter> filters;

  if(callbacks_.empty()) {
    // if no callbacks are registered, we set a filter that accepts all messages
    can_filter filter;
    filter.can_id   = 0;
    filter.can_mask = 0;
    filters.push_back(filter);
  }

  for(auto &callback : callbacks_) {
    can_filter filter;
    filter.can_id   = callback.first;
    filter.can_mask = callback.first | CAN_RTR_FLAG;
    filters.push_back(filter);
  }

  if(setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) < 0) {
    return Status::ExecutionError("CanDriver: Failed to set CAN filter/mask");
  }

  // set buffer
  if(setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0) {
    return Status::ExecutionError("CanDriver: Failed to set CAN buffer size  setsockopt failed");
  }

  // set Timeout
  timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = _timeout_us;
  if(setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))) {
    internall_close_can();
    return Status::ExecutionError("CanDriver: Set CAN socket option 'SO_RCVTIMEO' failed !");
  }

  // Bind the socket to the CAN interface
  _addr.can_family  = AF_CAN;
  _addr.can_ifindex = _ifr.ifr_ifindex;
  if(bind(socket_fd, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
    return Status::ExecutionError("CanDriver: Socket Bind Error");
  }

  bool in_callback_thread = can_perform_tasks_thread.get_id() == std::this_thread::get_id();

  run_thread_write.store(true);
  run_thread_perform_tasks.store(true);
  can_write_thread = std::thread(&CanDriver::handle_can_transmission, this);
  if(!in_callback_thread)
    can_perform_tasks_thread = std::thread(&CanDriver::handle_can_tasks, this);

  if(!can_write_thread.joinable()) {
    internall_close_can();
    return Status::ExecutionError("CanDriver: Failed to create CAN thread");
  }

  if(!in_callback_thread && !can_perform_tasks_thread.joinable()) {
    internall_close_can();
    return Status::ExecutionError("CanDriver: Failed to create CAN thread");
  }

  is_open = true;
  return Status::OK();
}

Status CanDriver::close_can() {
  return internall_close_can();
}

Status CanDriver::internall_close_can() {
  bool in_collback_close = can_perform_tasks_thread.get_id() == std::this_thread::get_id();
  if(in_collback_close) {
    run_thread_write.store(false);
  } else {
    run_thread_write.store(false);
    run_thread_perform_tasks.store(false);
    if(can_perform_tasks_thread.joinable()) {
      can_perform_tasks_thread.join();
    }
  }

  if(can_write_thread.joinable()) {
    can_write_thread.join();
  }

  if(socket_fd >= 0) {
    close(socket_fd);
    socket_fd = -1;
  }

  is_open = false;
  return Status::OK();
}

Status CanDriver::send_to_queue(const CanFrame &frame) {
  std::lock_guard<std::mutex> lock(queue_mutex);
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }

  if(frame.size > CANFD_MAX_DLEN) {
    return Status::Invalid("CAN frame size exceeds maximum data size");
  }

  if(frame.id == 0) {
    return Status::Invalid("CAN frame ID is invalid");
  }
  send_queue.push(frame);
  return Status::OK();
}

Status CanDriver::add_callback(uint32_t id, can_callback_type callback, void *args) {
  std::lock_guard<std::mutex> lock(interface_mutex);
  bool can_was_open = is_open;
  if(is_open) {
    internall_close_can();
    // return Status::Invalid("Cannot add callback after opening CAN socket");
  }

  callbacks_.find(id);
  if(callbacks_.find(id) != callbacks_.end()) {
    return Status::KeyError("Callback for ID already exists");
  }
  if(callback == nullptr) {
    return Status::Invalid("Callback function is null");
  }
  // Add the callback for the specified ID
  callbacks_[id] = std::make_pair(callback, args);

  if(can_was_open) {
    return open_can();
  }
  return Status::OK();
}

Status CanDriver::remove_callback(uint32_t id) {
  std::lock_guard<std::mutex> lock(interface_mutex);
  bool can_was_open = is_open;
  if(is_open) {
    internall_close_can();
    // return Status::Invalid("Cannot remove callback after opening CAN socket");
  }

  auto it = callbacks_.find(id);
  if(it == callbacks_.end())
    return Status::KeyError("Callback for ID not found");
  callbacks_.erase(it);

  if(can_was_open) {
    return open_can();
  }
  return Status::OK();
}

Status CanDriver::write_can_frame(const CanFrame &canframe) {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }
  auto frame = to_can_frame<can_frame>(canframe);

  // std::lock_guard<std::mutex> lock(can_mutex);
  ssize_t bytes_written = write(socket_fd, &frame, sizeof(frame));
  if(bytes_written < 0) {
    return Status::IOError("CanDriver: Failed to write CAN frame");
  }
  return Status::OK();
}

Result<CanFrame> CanDriver::read_can_frame() {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }
  // std::lock_guard<std::mutex> lock(can_mutex);
  can_frame frame;
  ssize_t bytes_read = read(socket_fd, &frame, sizeof(frame));
  if(bytes_read < 0) {
    return Status::IOError("CanDriver: Failed to read CAN frame");
  }
  return Result<CanFrame>::OK(from_can_frame<decltype(frame)>(frame));
}

Result<CanFrame> CanDriver::read_and_handle_frame() {
  // std::lock_guard<std::mutex> lock(queue_mutex);
  ARI_ASIGN_OR_RETURN(frame, read_can_frame());
  uint32_t id = frame.id | (frame.is_remote_request ? CAN_RTR_FLAG : 0);

  auto pending_it    = _pending_responses.find(id);
  bool enter_pending = pending_it != _pending_responses.end();
  if(!enter_pending) {
    pending_it    = _pending_responses.find(0);
    enter_pending = pending_it != _pending_responses.end();
  }
  if(enter_pending) {
    {
      std::lock_guard<std::mutex> lock(pending_it->second->mtx);
      pending_it->second->frame    = frame;
      pending_it->second->received = true;
    }
    pending_it->second->cv.notify_all();

    _pending_responses.erase(pending_it);
    return Result<CanFrame>::OK(std::move(frame));
  }


  auto it = callbacks_.find(id);
  if(it != callbacks_.end()) {
    auto callback = it->second.first;
    auto args     = it->second.second;
    callback(*this, frame, args);
  }
  return Result<CanFrame>::OK(std::move(frame));
}

Status CanDriver::send(const CanFrame &frame) {
  if(!is_open) {
    return Status::Invalid("CAN socket is not open");
  }

  if(frame.size > CANFD_MAX_DLEN) {
    return Status::Invalid("CAN frame size exceeds maximum data size");
  }

  if(frame.id == 0) {
    return Status::Invalid("CAN frame ID is invalid");
  }

  return send_to_queue(frame);
}

Result<CanFrame> CanDriver::send_await_response(const CanFrame &frame, uint32_t response_id, uint32_t timeout_ms) {
  PendingResponse pending_response;

  auto el = _pending_responses.find(response_id);
  if(el != _pending_responses.end()) {
    return Status::Invalid(
    "There is already a pending response for this CAN ID/ technically we shouldn't ever get here");
  }

  {
    std::lock_guard<std::mutex> lock(pending_response.mtx);
    _pending_responses[response_id] = &pending_response;
  }

  auto status = send(frame);
  if(!status.ok()) {
    _pending_responses.erase(response_id);
    return status;
  }

  std::unique_lock<std::mutex> lock(pending_response.mtx);
  auto success = pending_response.cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                              [&pending_response]() { return pending_response.received; });
  if(!success) {
    _pending_responses.erase(response_id);
    return Status::TimeOut("Timeout while waiting for CAN response");
  }
  return Result<CanFrame>::OK(std::move(pending_response.frame));
}

void CanDriver::handle_can_transmission() {
  while(run_thread_write) {

    // Check if there are any frames to send
    queue_mutex.lock();
    auto queue_empty = send_queue.empty();
    queue_mutex.unlock();

    if(queue_empty) {
      std::unique_lock<std::mutex> lock(queue_mutex);
      queue_condition.wait_for(lock, std::chrono::microseconds(100), [this]() { return !send_queue.empty(); });
      continue;
    }
    auto frame = send_queue.front();
    send_queue.pop();
    write_can_frame(frame);
  }
}

void CanDriver::handle_can_tasks() {
  while(run_thread_perform_tasks) {
    (void)read_and_handle_frame();
  }
}