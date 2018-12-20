// BSD 3-Clause License
//
// Copyright (c) 2018, Gregory Meyer
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef SWEEP_UTIL_TYPES_H
#define SWEEP_UTIL_TYPES_H

#include <cstdint>

#include <gsl/gsl_util>

namespace sweep::util {
inline namespace types {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using i8 = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using usize = std::size_t;
using isize = std::ptrdiff_t;

using f32 = float;
using f64 = double;

inline namespace literals {

constexpr u8 operator""_u8(unsigned long long x) noexcept {
    return gsl::narrow_cast<u8>(x);
}

constexpr u16 operator""_u16(unsigned long long x) noexcept {
    return gsl::narrow_cast<u16>(x);
}

constexpr u32 operator""_u32(unsigned long long x) noexcept {
    return gsl::narrow_cast<u32>(x);
}

constexpr u64 operator""_u64(unsigned long long x) noexcept {
    return gsl::narrow_cast<u64>(x);
}

constexpr i8 operator""_i8(unsigned long long x) noexcept {
    return gsl::narrow_cast<i8>(x);
}

constexpr i16 operator""_i16(unsigned long long x) noexcept {
    return gsl::narrow_cast<i16>(x);
}

constexpr i32 operator""_i32(unsigned long long x) noexcept {
    return gsl::narrow_cast<i32>(x);
}

constexpr i64 operator""_i64(unsigned long long x) noexcept {
    return gsl::narrow_cast<i64>(x);
}

constexpr usize operator""_usize(unsigned long long x) noexcept {
    return gsl::narrow_cast<usize>(x);
}

constexpr isize operator""_isize(unsigned long long x) noexcept {
    return gsl::narrow_cast<isize>(x);
}

constexpr f32 operator""_f32(long double x) noexcept {
    return gsl::narrow_cast<f32>(x);
}

constexpr f64 operator""_f64(long double x) noexcept {
    return gsl::narrow_cast<f64>(x);
}

} // inline namespace literals
} // inline namespace types
} // namespace sweep::util

#endif
