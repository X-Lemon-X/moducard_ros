#pragma once

#include <cstdint>


template <float Scale> struct FloatInt16_t {

  int16_t value;


  FloatInt16_t() : value(0) {
  }

  FloatInt16_t(float f) : value(static_cast<int16_t>(f / Scale)) {
  }

  operator float() const {
    return static_cast<float>(value) * Scale;
  }

  FloatInt16_t &operator=(float f) {
    value = static_cast<int16_t>(f / Scale);
    return *this;
  }
};