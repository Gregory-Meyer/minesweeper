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

#ifndef SWEEP_UTIL_STATE_H
#define SWEEP_UTIL_STATE_H

#include <sweep/util/types.h>

#include <cassert>
#include <functional>
#include <iosfwd>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>

namespace sweep::util {
namespace state {

class Unopened { };

class Opened {
public:
	constexpr explicit Opened(i8 x) : value_{ x } {
		if (!(x >= 0 && x <= 8)) {
			throw std::out_of_range{ "sweep::util::state::Opened::Opened()" };
		}
	}

	constexpr operator i8() const noexcept {
		return get();
	}

	constexpr i8 get() const noexcept {
		return value_;
	}

private:
	i8 value_;
};

class Mine { };

template <typename C, typename ...Ts>
struct IsInvocableUnionType : std::conjunction<std::is_invocable<C, Ts>...> { };

template <typename C, typename ...Ts>
inline constexpr bool IS_INVOCABLE_UNION = IsInvocableUnionType<C, Ts...>::value;

template <typename C, typename ...Ts>
struct IsNothrowInvocableUnionType : std::conjunction<std::is_nothrow_invocable<C, Ts>...> { };

template <typename C, typename ...Ts>
inline constexpr bool IS_NOTHROW_INVOCABLE_UNION = IsNothrowInvocableUnionType<C, Ts...>::value;

template <typename C, typename ...Ts>
struct CommonInvokeResultType : std::common_type<std::invoke_result_t<C, Ts>...> { };

template <typename C, typename ...Ts>
using CommonInvokeResult = typename CommonInvokeResultType<C, Ts...>::type;

template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };

template<class... Ts> Overload(Ts...) -> Overload<Ts...>;

class State {
public:
	constexpr explicit State(Unopened) noexcept : data_{ Unopened{ } } { }

	constexpr explicit State(Opened opened) noexcept : data_{ opened } { }

	constexpr explicit State(Mine) noexcept : data_{ Mine{ } } { }

	template <
		typename C, std::enable_if_t<IS_INVOCABLE_UNION<C, Unopened, Opened, Mine>, int> = 0
	>
	constexpr CommonInvokeResult<C, Unopened, Opened, Mine> visit(C &&callable) const
	noexcept(IS_NOTHROW_INVOCABLE_UNION<C, Unopened, Opened, Mine>) {
		return std::visit(std::forward<C>(callable), data_);
	}

	constexpr bool is_unopened() const noexcept {
		return std::holds_alternative<Unopened>(data_);
	}

	constexpr bool is_opened() const noexcept {
		return std::holds_alternative<Opened>(data_);
	}

	constexpr bool is_mine() const noexcept {
		return std::holds_alternative<Mine>(data_);
	}

	constexpr std::optional<Opened> as_opened() const noexcept {
		return visit(Overload{
			[](Opened o) -> std::optional<Opened> { return { o }; },
			[](auto) -> std::optional<Opened> { return std::nullopt; }
		});
	}

private:
	std::variant<Unopened, Opened, Mine> data_;
};

std::ostream& operator<<(std::ostream &os, Unopened);

std::ostream& operator<<(std::ostream &os, Opened o);

std::ostream& operator<<(std::ostream &os, Mine);

std::ostream& operator<<(std::ostream &os, State state);

} // namespace state

using state::Unopened; using state::Opened; using state::Mine; using state::State;
using state::operator<<;

} // namespace sweep::util

#endif
