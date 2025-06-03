#include "ExactSeidelSolverPoint.hh"

#include <rich-log/log.hh>

#include <integer-plane-geometry/any_point.hh>
#include <integer-plane-geometry/are_parallel.hh>
#include <integer-plane-geometry/classify.hh>

namespace
{
/// returns 1, if the line orientation and the plane normal match
/// returns -1, if the line orientation and the plane normal are opposite
/// returns 0, if the line is parallel to the plane
template <class geometry_t>
int orientation(ipg::line<geometry_t> const& line, ipg::plane<geometry_t> const& plane)
{
    static constexpr int bits_normal = geometry_t::bits_normal;
    static constexpr int bits_nn = ipg::line<geometry_t>::bits_nn;
    static constexpr int bits_dot = bits_normal + bits_nn + 2;

    auto const dir = line.direction();
    auto const normal = plane.normal();

    auto const dot = ipg::mul<bits_dot>(dir.x, normal.x) + //
                     ipg::mul<bits_dot>(dir.y, normal.y) + //
                     ipg::mul<bits_dot>(dir.z, normal.z);

    return tg::sign(dot);
}
}

void mk::ExactSeidelSolverPoint::set_planes(cc::span<plane_t const> planes)
{
    // reset
    m_solution = {};

    // allocate memory
    m_mapping.resize(planes.size());
    m_planes.resize(planes.size());

    // compute mapping:
    for (auto i = 0; i < int(planes.size()); ++i)
        m_mapping[i] = i;

    // shuffle (randomness is important for the seidel solver!)
    tg::shuffle(m_rng, m_mapping);

    // copy planes
    for (auto i = 0u; i < planes.size(); ++i)
        m_planes[i] = planes[m_mapping[i]];
}

cc::array<int, 3> mk::ExactSeidelSolverPoint::solution_planes() const
{
    return {
        m_solution.plane_idx_0 >= 0 ? m_mapping[m_solution.plane_idx_0] : -1, //
        m_solution.plane_idx_1 >= 0 ? m_mapping[m_solution.plane_idx_1] : -1, //
        m_solution.plane_idx_2 >= 0 ? m_mapping[m_solution.plane_idx_2] : -1, //

    };
}

mk::ExactSeidelSolverPoint::state mk::ExactSeidelSolverPoint::solve_3D_problem(cc::span<plane_t const> planes)
{
    m_solution.reset();
    for (auto pi = 0; pi < int(planes.size()); ++pi)
    {
        if (m_should_stop)
        {
            return state::infeasible; // might not actually be infeasible, but does not matter at this point
        }

        auto const plane = m_planes[pi];

        if (m_solution.is_point())
        {
            if (ipg::classify(m_solution.position, plane) <= 0)
                continue;
        }

        else if (m_solution.is_line())
        {
            if (ipg::are_parallel(plane, m_solution.line))
            {
                if (ipg::classify(ipg::any_point(m_solution.line), plane) <= 0)
                    continue;
            }
            else
            {
                m_solution.append(pi, plane);
                continue;
            }
        }
        else if (m_solution.is_plane())
        {
            if (ipg::are_parallel(m_solution.plane, plane))
            {
                if (ipg::classify(ipg::any_point(m_solution.plane), plane) <= 0)
                    continue;
            }
            else
            {
                m_solution.append(pi, plane);
                continue;
            }
        }
        else
        {
            CC_ASSERT(m_solution.is_space());
            m_solution.append(pi, plane);
            continue;
        }

        // solution not valid anymore, solve 2d problem
        auto state_2d = solve_2D_problem(planes.first(pi), pi);
        if (state_2d == state::infeasible)
            return state::infeasible;
    }


    return state::has_solution;
}

mk::ExactSeidelSolverPoint::state mk::ExactSeidelSolverPoint::solve_2D_problem(cc::span<plane_t const> planes, int fixed_plane_3D_idx)
{
    m_solution.reset();
    auto const fixed_plane = m_planes[fixed_plane_3D_idx];
    m_solution.append(fixed_plane_3D_idx, fixed_plane);

    for (auto pi = 0; pi < int(planes.size()); ++pi)
    {
        if ((pi + 1) % 1000 == 0 && m_should_stop)
        {
            return state::infeasible; // might not actually be infeasible, but takes the direct return path
        }

        auto const plane = planes[pi];

        if (m_solution.is_point())
        {
            if (ipg::classify(m_solution.position, plane) <= 0)
                continue;
        }

        else if (m_solution.is_line())
        {
            if (ipg::are_parallel(plane, m_solution.line))
            {
                auto const pt = ipg::any_point(m_solution.line);
                if (ipg::classify(pt, plane) <= 0)
                    continue;
            }
            else
            {
                m_solution.append(pi, plane);
                continue;
            }
        }
        else
        {
            CC_ASSERT(m_solution.is_plane());
            if (ipg::are_parallel(m_solution.plane, plane))
            {
                if (ipg::classify(ipg::any_point(m_solution.plane), plane) <= 0)
                    continue;
            }
            else
            {
                m_solution.append(pi, plane);
                continue;
            }
        }

        if (ipg::are_parallel(plane, fixed_plane))
        {
            if (ipg::classify(ipg::any_point(fixed_plane), plane) == 1)
                return state::infeasible;
        }

        // line solution not valid anymore, build 1d solution
        auto state_1d = solve_1D_problem(cc::span(m_planes).first(pi), fixed_plane_3D_idx, pi);
        if (state_1d == state::infeasible)
            return state::infeasible;
    }

    return state::has_solution;
}

mk::ExactSeidelSolverPoint::state mk::ExactSeidelSolverPoint::solve_1D_problem(cc::span<plane_t const> planes, int fixed_plane_3D_idx, int fixed_plane_2D_idx)
{
    // invalidate solution:
    m_solution.reset();
    m_solution.append(fixed_plane_3D_idx, m_planes[fixed_plane_3D_idx]);
    m_solution.append(fixed_plane_2D_idx, m_planes[fixed_plane_2D_idx]);

    struct interval
    {
        int left_idx = -1;
        int right_idx = -1;
        point4_t left_point;
        point4_t right_point;
        int left_orientation = 0;
        int right_orientation = 0;

        bool is_line() const { return left_idx < 0; }
        bool is_one_sided() const { return left_idx >= 0 && right_idx < 0; }
        bool is_closed() const { return left_idx >= 0 && right_idx >= 0; }
    } interval;

    for (auto pi = 0; pi < int(planes.size()); ++pi)
    {
        auto const plane = planes[pi];

        if (interval.is_closed())
        {
            CC_ASSERT(interval.left_orientation != interval.right_orientation);
            CC_ASSERT(interval.left_orientation != 0);
            CC_ASSERT(interval.right_orientation != 0);
            // check both interval bounds
            auto const c_left = ipg::classify(interval.left_point, plane);
            auto const c_right = ipg::classify(interval.right_point, plane);

            if (c_left == 1)
            {
                if (c_right == 1)
                {
                    return state::infeasible;
                }
                else
                {
                    // update left interval
                    interval.left_idx = pi;
                    interval.left_point = ipg::intersect(m_solution.line, plane);
                    CC_ASSERT(interval.left_orientation == orientation(m_solution.line, plane));
                }
            }
            else
            {
                CC_ASSERT(c_left <= 0);
                if (c_right == 1)
                {
                    // update right interval
                    interval.right_idx = pi;
                    interval.right_point = ipg::intersect(m_solution.line, plane);
                    CC_ASSERT(interval.right_orientation == orientation(m_solution.line, plane));
                }
                // else noop, interval is still valid
            }
        }
        else if (interval.is_one_sided())
        {
            auto const c = ipg::classify(interval.left_point, plane);
            auto const o = orientation(m_solution.line, plane);

            if (o == 0)
            {
                // parallel to line
                if (c > 0)
                    return state::infeasible;
            }
            else if (c == 1)
            {
                if (o == interval.left_orientation)
                {
                    // new plane is a tighter left bound
                    interval.left_idx = pi;
                    interval.left_orientation = o;
                    interval.left_point = ipg::intersect(m_solution.line, plane);
                }
                else
                {
                    return state::infeasible;
                }
            }
            else if (o != interval.left_orientation)
            {
                // found right side
                interval.right_idx = pi;
                interval.right_orientation = o;
                interval.right_point = ipg::intersect(m_solution.line, plane);
            }
            // else  noop, already tight bound
        }
        else
        {
            CC_ASSERT(interval.is_line());
            auto const o = orientation(m_solution.line, plane);
            if (o == 0)
            {
                // parallel
                auto const c = ipg::classify(ipg::any_point(m_solution.line), plane);
                if (c == 1)
                    return state::infeasible;
            }
            else
            {
                CC_ASSERT(interval.left_idx < 0);
                interval.left_idx = pi;
                interval.left_orientation = o;
                CC_ASSERT(interval.left_orientation != 0);
                interval.left_point = ipg::intersect(m_solution.line, plane);
            }
        }
    }

    if (interval.left_idx >= 0)
        m_solution.append(interval.left_idx, m_planes[interval.left_idx]);

    return state::has_solution;
}

mk::ExactSeidelSolverPoint::state mk::ExactSeidelSolverPoint::solve()
{
    m_should_stop = false;
    return solve_3D_problem(m_planes);
}
