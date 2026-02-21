#pragma once
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
#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace mcan {


class TimingObject {
public:
  TimingObject(float frequency_hz)
  : _interval(static_cast<int64_t>(1000'000.0 / frequency_hz)), _last_trigger(std::chrono::steady_clock::now()) {
  }
  TimingObject() : _interval(0), _last_trigger(std::chrono::steady_clock::now()) {
  }
  TimingObject(TimingObject &&t) : _interval(t._interval), _last_trigger(t._last_trigger) {
  }


  // Get elapsed time since last trigger
  double elapsed_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - _last_trigger).count();
  }

  void reset() {
    _last_trigger = std::chrono::steady_clock::now();
  }

  std::chrono::nanoseconds _interval;
  std::chrono::steady_clock::time_point _last_trigger;
};


class FrequencyTimer {
public:
  FrequencyTimer(){};

  // Check if it's time to trigger, auto-resets
  static bool should_trigger_and_reset(TimingObject &timer) {
    auto now     = std::chrono::steady_clock::now();
    auto elapsed = now - timer._last_trigger;
    if(elapsed >= timer._interval) {
      timer._last_trigger = now;
      return true;
    }
    return false;
  }

  // Check if it's time to trigger, does not reset
  static bool should_trigger(TimingObject &timer) {
    auto now     = std::chrono::steady_clock::now();
    auto elapsed = now - timer._last_trigger;
    return elapsed >= timer._interval;
  }
};

} // namespace mcan