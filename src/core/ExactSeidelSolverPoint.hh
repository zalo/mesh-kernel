#pragma once

#include <atomic>

#include <clean-core/pair.hh>
#include <clean-core/span.hh>
#include <clean-core/vector.hh>

#include <integer-plane-geometry/geometry.hh>
#include <integer-plane-geometry/intersect.hh>
#include <integer-plane-geometry/line.hh>
#include <integer-plane-geometry/plane.hh>
#include <integer-plane-geometry/point.hh>
#include "integer-plane-geometry/any_point.hh"

namespace mk
{
class ExactSeidelSolverPoint
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using plane_t = geometry_t::plane_t;
    using point4_t = geometry_t::point4_t;
    using line_t = ipg::line<geometry_t>;

    enum class state
    {
        infeasible,
        has_solution,
        // minimal, // without an objective we cannot really say what minimal means
        unbounded, // without an objective it's much more difficult to differentiate between unbounded and minimal
        ambiguous
    };

    struct solution
    {
        int plane_idx_0 = -1;
        int plane_idx_1 = -1;
        int plane_idx_2 = -1;

        plane_t plane;
        line_t line;
        point4_t position;

        void reset()
        {
            plane_idx_0 = -1;
            plane_idx_1 = -1;
            plane_idx_2 = -1;
        }

        void append(int index, plane_t const& new_plane)
        {
            if (plane_idx_0 < 0)
            {
                plane_idx_0 = index;
                plane = new_plane;
                CC_ASSERT(plane.is_valid());
            }
            else if (plane_idx_1 < 0)
            {
                plane_idx_1 = index;
                line = ipg::intersect(plane, new_plane);
                CC_ASSERT(line.is_valid());
            }
            else
            {
                CC_ASSERT(plane_idx_2 < 0 && "Cannot append plane to full plane!");
                plane_idx_2 = index;
                position = ipg::intersect(line, new_plane);
                CC_ASSERT(position.is_valid());
            }
        }

        bool is_space() const { return plane_idx_0 < 0; }
        bool is_point() const { return plane_idx_2 >= 0; }
        bool is_line() const { return plane_idx_2 == -1 && plane_idx_1 >= 0; }
        bool is_plane() const { return plane_idx_0 >= 0 && plane_idx_1 < 0; }

        point4_t any_point() const
        {
            if (is_point())
                return position;
            if (is_line())
                return ipg::any_point(line);
            if (is_plane())
                return ipg::any_point(plane);
            CC_UNREACHABLE("Unhandeled case!");
        }

        bool is_unbounded() const { return plane_idx_0 == -1 || plane_idx_1 == -1 || plane_idx_2 == -1; }
    };


public: // API
    /// set the 3d planes that define the problem
    void set_planes(cc::span<plane_t const> planes);

    /// solve the given problem
    state solve();

    /// once solved, returns the indices of the planes that define the solution segment in the original input
    cc::array<int, 3> solution_planes() const;

    solution const& get_solution() { return m_solution; }

    void stop() { m_should_stop = true; }

private: // member
    // state m_state = state::ambiguous;
    tg::rng m_rng;
    cc::vector<int> m_mapping;
    cc::vector<plane_t> m_planes;

    std::atomic<bool> m_should_stop;

    solution m_solution;

private: // helper methods
    state solve_3D_problem(cc::span<plane_t const> planes);

    state solve_2D_problem(cc::span<plane_t const> planes, int fixed_plane_3D_idx);

    state solve_1D_problem(cc::span<plane_t const> planes, int fixed_plane_3D_idx, int fixed_plane_2D_idx);
};
}
