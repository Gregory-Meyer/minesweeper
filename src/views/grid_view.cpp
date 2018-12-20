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

#include <sweep/views/grid_view.h>

#include <cassert>
#include <iostream>
#include <type_traits>

namespace sweep::views::grid_view {

struct GridView::Helpers {
    static constexpr void ensure_invariants(const GridView &gv) noexcept {
        assert(gv.rows_ >= 2);
        assert(gv.cols_ >= 2);
        assert(static_cast<Grid::size_type>(gv.rows_ * gv.cols_) == gv.grid_.size());
    }

    static constexpr Grid::size_type get_flat_index(const GridView &gv, const Point &p) noexcept {
        ensure_invariants(gv);

        assert(p.i >= 0 && p.i <= gv.rows_);
        assert(p.j >= 0 && p.j < gv.cols_);

        return static_cast<Grid::size_type>(p.i * gv.cols_ + p.j);
    }

    static constexpr Grid::reference get(GridView &gv, const Point &p) noexcept {
        ensure_invariants(gv);

        assert(p.i >= 0 && p.i <= gv.rows_);
        assert(p.j >= 0 && p.j < gv.cols_);

        return gv.grid_[get_flat_index(gv, p)];
    }

    static constexpr Grid::const_reference get(const GridView &gv, const Point &p) noexcept {
        ensure_invariants(gv);

        assert(p.i >= 0 && p.i <= gv.rows_);
        assert(p.j >= 0 && p.j < gv.cols_);

        return gv.grid_[get_flat_index(gv, p)];
    }
};

GridView::GridView(isize num_rows, isize num_cols)
: grid_(static_cast<Grid::size_type>(num_rows * num_cols), State{ Unopened { } }),
  rows_{ num_rows }, cols_{ num_cols } {
    Helpers::ensure_invariants(*this);
}

void GridView::do_update(const Point &location, const State &state) {
    Helpers::ensure_invariants(*this);

    assert(location.i >= 0 && location.i < rows_);
    assert(location.j >= 0 && location.j < cols_);

    Helpers::get(*this, location) = state;
}

std::ostream& GridView::do_draw_on(std::ostream &os) const {
    Helpers::ensure_invariants(*this);

    os << u8"┌";

    for (isize j = 0; j < cols_; ++j) {
        os << u8"─";
    }

    os << u8"┐\n";

    const auto print_cell = [this, &os](isize i, isize j) {
        Helpers::get(*this, { i, j }).visit([&os](const auto &x) {
            using T = std::decay_t<decltype(x)>;

            if constexpr (std::is_same_v<T, Unopened>) {
                os << ' ';
            } else if constexpr (std::is_same_v<T, Opened>) {
                os << static_cast<int>(x.get());
            } else if constexpr (std::is_same_v<T, Mine>) {
                os << 'x';
            }
        });
    };

    for (isize i = 0; i < rows_; ++i) {
        os << u8"│";

        for (isize j = 0; j < cols_; ++j) {
            print_cell(i, j);
        }

        os << u8"│\n";
    }

    os << u8"└";

    for (isize j = 0; j < cols_; ++j) {
        os << u8"─";
    }

    os << u8"┘";

    return os;
}

} // namespace sweep::views::grid_view
