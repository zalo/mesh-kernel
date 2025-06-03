#pragma once
#include <clean-core/array.hh>
#include <clean-core/pair.hh>
#include <clean-core/span.hh>

#include <rich-log/log.hh>

#include <typed-geometry/feature/basic.hh>
#include <typed-geometry/tg-lean.hh>

#include <integer-plane-geometry/geometry.hh>
#include <polymesh/Mesh.hh>
#include <typed-geometry/functions/vector/dot.hh>

// default planes for kdop as follows:
// taken from https://github.com/flexible-collision-library/fcl/blob/master/include/fcl/math/bv/kDOP.h
// For K = 16, the planes are 6 AABB planes and 10 diagonal planes that cut off some space of the edges:
// (-1,0,0) and (1,0,0)
// (0,-1,0) and (0,1,0)
// (0,0,-1) and (0,0,1)
// (-1,-1,0) and (1,1,0)
// (-1,0,-1) and (1,0,1)
// (0,-1,-1) and (0,1,1)
// (-1,1,0) and (1,-1,0)
// (-1,0,1) and (1,0,-1)
// For K = 18, the planes are 6 AABB planes and 12 diagonal planes that cut off some space of the edges:
// (-1,0,0) and (1,0,0)
// (0,-1,0) and (0,1,0)
// (0,0,-1) and (0,0,1)
// (-1,-1,0) and (1,1,0)
// (-1,0,-1) and (1,0,1)
// (0,-1,-1) and (0,1,1)
// (-1,1,0) and (1,-1,0)
// (-1,0,1) and (1,0,-1)
// (0,-1,1) and (0,1,-1)
// For K = 18, the planes are 6 AABB planes and 18 diagonal planes that cut off some space of the edges:
// (-1,0,0) and (1,0,0)
// (0,-1,0) and (0,1,0)
// (0,0,-1) and (0,0,1)
// (-1,-1,0) and (1,1,0)
// (-1,0,-1) and (1,0,1)
// (0,-1,-1) and (0,1,1)
// (-1,1,0) and (1,-1,0)
// (-1,0,1) and (1,0,-1)
// (0,-1,1) and (0,1,-1)
// (-1, -1, 1) and (1, 1, -1)
// (-1, 1, -1) and (1, -1, 1)
// (1, -1, -1) and (-1, 1, 1)

template <size_t K, class ScalarT>
struct k_dop
{
public:
    using vec_t = tg::vec<3, ScalarT>;
    using pos_t = tg::pos<3, ScalarT>;

    k_dop();

    /* data */
    cc::array<vec_t, K> axis;
    cc::array<pm::vertex_handle, K> vertices_min;
    cc::array<pm::vertex_handle, K> vertices_max;
    cc::array<ScalarT, K> distance_min;
    cc::array<ScalarT, K> distance_max;

    /// returns the distance between the given point and the given axis
    /// calculated dot(point, axis)
    ScalarT get_distance(size_t axis_idx, pos_t const& point) const { return tg::dot(point, axis[axis_idx]); }

    /// sets the vertices of the kdop
    void initialize_from_positions(pm::vertex_attribute<pos_t> const& positions);

    void update(cc::span<pm::vertex_handle const> cut_vertices, pm::vertex_attribute<pos_t> const& positions)
    {
        CC_ASSERT(!cut_vertices.empty());

        cc::array<bool, K> min_needs_update;
        cc::array<bool, K> max_needs_update;

        for (auto i = 0; i < 3; ++i)
        {
            min_needs_update[i] = vertices_min[i].is_removed();
            if (min_needs_update[i])
                distance_min[i] = positions[cut_vertices[0]][i];

            max_needs_update[i] = vertices_max[i].is_removed();
            if (max_needs_update[i])
                distance_max[i] = positions[cut_vertices[0]][i];
        }

        for (auto const v : cut_vertices)
        {
            auto const& p = positions[v];
            for (auto d = 0; d < 3; ++d)
            {
                auto const val = p[d];
                if (min_needs_update[d])
                {
                    if (p[d] < distance_min[d])
                    {
                        distance_min[d] = p[d];
                        vertices_min[d] = v;
                    }
                }

                if (max_needs_update[d])
                {
                    if (p[d] > distance_max[d])
                    {
                        distance_max[d] = p[d];
                        vertices_max[d] = v;
                    }
                }
            }
        }

        // assure the bounds are conservative
        for (auto i = 0; i < 3; ++i)
        {
            min_needs_update[i] = vertices_min[i].is_removed();
            if (min_needs_update[i])
                distance_min[i] -= 1;

            max_needs_update[i] = vertices_max[i].is_removed();
            if (max_needs_update[i])
                distance_max[i] += 1;
        }
    }

    /// returns K
    size_t size() const { return K; }
};

template <size_t K, class ScalarT>
void k_dop<K, ScalarT>::initialize_from_positions(pm::vertex_attribute<pos_t> const& positions)
{
    // init distances
    for (size_t i = 0; i < K; ++i)
    {
        distance_min[i] = std::numeric_limits<ScalarT>::max();
        distance_max[i] = -std::numeric_limits<ScalarT>::max(); // std::numeric_limits<ScalarT>::min() for float types is the smallest positive number!
    }

    auto const& m = positions.mesh();
    for (auto const v : m.vertices())
    {
        auto const& p = positions[v];
        for (size_t i = 0; i < K; ++i)
        {
            auto const d = get_distance(i, p);
            if (d < distance_min[i])
            {
                distance_min[i] = d;
                vertices_min[i] = v;
            }
            if (d > distance_max[i])
            {
                distance_max[i] = d;
                vertices_max[i] = v;
            }
        }
    }

    // assure the bounds are conservative
    for (size_t i = 0; i < K; ++i)
    {
        distance_min[i] -= 1;
        distance_max[i] += 1;
    }
}

template <size_t K, class ScalarT>
k_dop<K, ScalarT>::k_dop()
{
    if (K >= 3)
    {
        axis[0] = vec_t(1, 0, 0);
        axis[1] = vec_t(0, 1, 0);
        axis[2] = vec_t(0, 0, 1);

        if (K >= 8)
        {
            axis[3] = vec_t(1, 1, 0);
            axis[4] = vec_t(1, 0, 1);
            axis[5] = vec_t(0, 1, 1);
            axis[6] = vec_t(1, -1, 0);
            axis[7] = vec_t(1, 0, -1);

            if (K >= 9)
            {
                axis[8] = vec_t(0, 1, -1);

                if (K >= 12)
                {
                    axis[9] = vec_t(1, 1, -1);
                    axis[10] = vec_t(1, -1, 1);
                    axis[11] = vec_t(-1, 1, 1);
                }
            }
        }
    }
    else
    {
        CC_UNREACHABLE_SWITCH_WORKAROUND(k);
    }
}

/// specialization for aabb
template <>
struct k_dop<3, int>
{
    using vec_t = tg::vec<3, int>;
    using pos_t = tg::pos<3, int>;

    tg::iaabb3 aabb;
    cc::array<pm::vertex_handle, 3> vertices_min;
    cc::array<pm::vertex_handle, 3> vertices_max;

    void initialize_from_positions(pm::vertex_attribute<tg::ipos3> const& positions)
    {
        auto const& m = positions.mesh();
        auto const any_v = m.vertices().first();
        vertices_min[0] = any_v;
        vertices_min[1] = any_v;
        vertices_min[2] = any_v;
        vertices_max[0] = any_v;
        vertices_max[1] = any_v;
        vertices_max[2] = any_v;


        aabb.min = positions[any_v];
        aabb.max = positions[any_v];

        for (auto const v : m.vertices())
        {
            auto const& p = positions[v];
            for (auto d = 0; d < 3; ++d)
            {
                if (p[d] < aabb.min[d])
                {
                    aabb.min[d] = p[d];
                    vertices_min[d] = v;
                }
                if (p[d] > aabb.max[d])
                {
                    aabb.max[d] = p[d];
                    vertices_max[d] = v;
                }
            }
        }

        aabb.min -= 3;
        aabb.max += 3;

        using geometry_t = ipg::geometry<26, 55>;

        CC_ASSERT(tg::abs(aabb.min.x) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.min.y) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.min.z) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.x) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.y) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.z) <= (tg::i64(1) << geometry_t::bits_position));
    }

    void update(cc::span<pm::vertex_handle const> cut_vertices, pm::vertex_attribute<tg::dpos3> const& positions)
    {
        if (cut_vertices.empty())
            return; // nothing to do

        CC_ASSERT(cut_vertices[0].mesh == vertices_min[0].mesh);

        CC_ASSERT(!cut_vertices.empty());

        cc::array<bool, 3> min_needs_update;
        cc::array<bool, 3> max_needs_update;

        tg::daabb3 daabb = tg::daabb3(aabb);

        auto any_needs_update = false;
        for (auto i = 0; i < 3; ++i)
        {
            min_needs_update[i] = vertices_min[i].is_removed();
            if (min_needs_update[i])
            {
                any_needs_update = true;
                daabb.min[i] = positions[cut_vertices[0]][i];
            }

            max_needs_update[i] = vertices_max[i].is_removed();
            if (max_needs_update[i])
            {
                any_needs_update = true;
                daabb.max[i] = positions[cut_vertices[0]][i];
            }
        }

        if (!any_needs_update)
            return;

        for (auto const v : cut_vertices)
        {
            for (auto d = 0; d < 3; ++d)
            {
                auto const& p = positions[v];

                if (min_needs_update[d])
                {
                    auto min_val = tg::ifloor(p[d] - 1);
                    if (min_val < daabb.min[d])
                    {
                        daabb.min[d] = min_val;
                        vertices_min[d] = v;
                    }
                }

                if (max_needs_update[d])
                {
                    auto max_val = tg::iceil(p[d] + 1);
                    if (max_val > daabb.max[d])
                    {
                        daabb.max[d] = max_val;
                        vertices_max[d] = v;
                    }
                }
            }
        }


        // round conservatively
        auto new_aabb = tg::iaabb3(tg::ipos3(tg::ifloor(daabb.min)), //
                                   tg::ipos3(tg::iceil(daabb.max)));

        // but don't make the new aabb larger than before
        aabb.min.x = tg::max(aabb.min.x, new_aabb.min.x);
        aabb.min.y = tg::max(aabb.min.y, new_aabb.min.y);
        aabb.min.z = tg::max(aabb.min.z, new_aabb.min.z);

        aabb.max.x = tg::min(aabb.max.x, new_aabb.max.x);
        aabb.max.y = tg::min(aabb.max.y, new_aabb.max.y);
        aabb.max.z = tg::min(aabb.max.z, new_aabb.max.z);

        using geometry_t = ipg::geometry<26, 55>;

        CC_ASSERT(tg::abs(aabb.min.x) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.min.y) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.min.z) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.x) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.y) <= (tg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(aabb.max.z) <= (tg::i64(1) << geometry_t::bits_position));
    }

    size_t size() const { return 3; }
};
