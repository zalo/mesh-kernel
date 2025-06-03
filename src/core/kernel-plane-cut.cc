#include "kernel-plane-cut.hh"

#include <clean-core/indices_of.hh>
#include <clean-core/set.hh>
#include <clean-core/vector.hh>

#include <clean-ranges/algorithms/all.hh>

#include <reflector/to_string.hh>

#include <rich-log/log.hh>

#include <typed-geometry/functions/basic/scalar_math.hh>
#include <typed-geometry/tg-lean.hh>

#include <polymesh/Mesh.hh>
#include <polymesh/algorithms/triangulate.hh>
#include <polymesh/detail/union_find.hh>
#include <polymesh/objects/cube.hh>
#include <polymesh/properties.hh>
#include <polymesh/std/hash.hh>

#include <ctracer/trace-config.hh>
#include <ctracer/trace.hh>

#include <integer-plane-geometry/are_parallel.hh>
#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/intersect.hh>
#include <integer-plane-geometry/plane.hh>
#include <integer-plane-geometry/point.hh>

#if defined(MK_TBB_ENABLED)
#include <tbb/tbb.h>
#endif

// internal
#include <core/kdop.hh>

namespace
{
//* taken from https://johannesugb.github.io/cpu-programming/tools/floating-point-epsilon-calculator/
template <typename T>
T precision_for(T reference)
{
    T more = std::nextafter(reference, std::numeric_limits<T>::infinity());
    T less = std::nextafter(reference, -std::numeric_limits<T>::infinity());
    T precision = std::max(more - reference, reference - less);
    return precision;
}

template <typename ScalarT>
ScalarT to_cgal(tg::fixed_int<2> v)
{
    tg::fixed_uint<2> uv(v);
    uint64_t s = sign(v);
    ScalarT r = ScalarT(uint64_t(uv.d[1]));
    r = (uint64_t(1) << 63) * 2;
    r += uint64_t(uv.d[0]);
    r *= s;
    return r;
}
}

namespace mk
{

KernelPlaneCut::KernelPlaneCut(pm::vertex_attribute<pos_t> const& input_positions, kernel_options const& options)
{
    compute_kernel(input_positions, options);
}


void KernelPlaneCut::compute_kernel(pm::vertex_attribute<pos_t> const& input_positions, kernel_options const& options)
{
    reset();

    m_options = options;

    m_benchmark_data.input_faces = input_positions.mesh().faces().size();

    {
        TRACE("dummy");
    }

    {
        TRACE("complete kernel construction");

        init_input_planes(input_positions);
        init_edge_state(input_positions);

        if (is_convex())
        {
            m_benchmark_data.is_convex = true;
            m_benchmark_data.convex_contribution_kernel = m_benchmark_data.input_faces;
            m_benchmark_data.kernel_faces = m_benchmark_data.input_faces;
            m_benchmark_data.total_planes = m_benchmark_data.input_faces;
            m_has_kernel = true;
            m_input_is_convex = true;
            return;
        }

        m_cutting_planes.reserve(input_positions.mesh().faces().size());

        if (m_options.use_unordered_set)
            init_cutting_planes_uset(input_positions);
        else
            init_cutting_planes_flood_fill(input_positions);


        CC_ASSERT(m_cutting_planes.size() == m_face_of_plane.size());

        m_benchmark_data.total_planes = m_cutting_planes.size();
        m_benchmark_data.number_concave_planes = m_number_concave_planes;

        if (m_options.parallel_exact_lp)
        {
            m_exact_seidel_solver_result = std::async(std::launch::async,
                                                      [this]()
                                                      {
                                                          m_exact_seidel_solver.set_planes(m_cutting_planes);
                                                          return m_exact_seidel_solver.solve();
                                                      });
        }

        init_supporting_structure(input_positions);

        compute_mesh_kernel();
    }

    LOGD(Default, Info, "number of cutting planes: %s", m_cutting_planes.size());

    if (!m_has_kernel)
    {
        LOGD(Default, Info, "kernel is empty!");
        m_mesh.clear();
    }

    if (!pm::is_closed_mesh(m_mesh))
    {
        LOGD(Default, Info, "result mesh not closed!");
    }

    if (m_options.triangulate)
        pm::triangulate_naive(m_mesh);

    // stats

    if (m_has_kernel)
    {
        m_benchmark_data.kernel_faces = m_mesh.faces().size();
        for (auto const f : m_mesh.faces())
        {
            auto const orig_f = m_input_face[f];
            if (!orig_f.is_valid())
                continue;

            if (orig_f.edges().any(
                    [&](pm::edge_handle e)
                    {
                        return m_input_edge_state[e] == edge_state::concave ||  //
                               m_input_edge_state[e] == edge_state::boundary || //
                               m_input_edge_state[e] == edge_state::degenerate;
                    }))
            {
                m_benchmark_data.concave_contribution_kernel++;
            }
            else
            {
                m_benchmark_data.convex_contribution_kernel++;
            }
        }
    }
}

void KernelPlaneCut::reset()
{
    m_cutting_planes.clear();
    m_face_of_plane.clear();

    m_has_kernel = false;
    m_input_is_convex = true; // it's convex until we find an edge that says otherwise
    m_number_concave_planes = 0;

    m_benchmark_data = {};

    m_3dop = {};
    m_8dop = {};
    m_9dop = {};
    m_12dop = {};
    m_c0_vertices.clear();

    m_has_queried_future = false;
    m_is_infeasible = false;
}


void KernelPlaneCut::init_point4_position(pm::vertex_attribute<pos_t> const& positions)
{
    for (auto v : m_mesh.vertices())
    {
        m_position_point4[v] = positions[v];
        m_position_dpos[v] = tg::dpos3(positions[v]);
    }
}


bool KernelPlaneCut::is_convex() { return m_input_is_convex; }


bool KernelPlaneCut::has_trivial_solution()
{
    if (is_convex())
    {
        LOGD(Default, Info, "input mesh is convex");
        m_input_is_convex = true;
        m_has_kernel = true;
        return true;
    }
    return false;
}

void KernelPlaneCut::init_cutting_planes_flood_fill(pm::vertex_attribute<pos_t> const& positions)
{
    m_cutting_planes.clear();
    m_face_of_plane.clear();

    // TRACE();
    //* test example 113868.obj
    //* since we need to classify a vertex of every face we precompute the vertex points
    auto const& mesh = positions.mesh();

    CC_ASSERT(mesh.is_compact());

    // do union-find to merge coplanar regions
    auto union_find = pm::detail::disjoint_set(mesh.faces().size());
    for (auto const e : mesh.edges())
    {
        if (m_input_edge_state[e] != edge_state::planar)
            continue;
        union_find.do_union(e.faceA().idx.value, e.faceB().idx.value);
    }

    // collect planes adjacent to concave faces
    auto visited = pm::face_attribute<bool>(mesh, false);
    for (auto const e : mesh.edges())
    {
        if (m_input_edge_state[e] == edge_state::convex || m_input_edge_state[e] == edge_state::planar)
            continue;

        auto const face_a = e.faceA();
        auto const face_b = e.faceB();

        auto const rep_a = mesh.faces()[union_find.find(face_a.idx.value)];
        auto const rep_b = mesh.faces()[union_find.find(face_b.idx.value)];

        if (!visited[rep_a])
        {
            if (m_input_plane[rep_a].is_valid())
            {
                m_cutting_planes.push_back(m_input_plane[rep_a]);
                m_face_of_plane.push_back(rep_a);
            }
            visited[rep_a] = true;
        }

        if (!visited[rep_b])
        {
            if (m_input_plane[rep_b].is_valid())
            {
                m_cutting_planes.push_back(m_input_plane[rep_b]);
                m_face_of_plane.push_back(rep_b);
            }
            visited[rep_b] = true;
        }
    }

    m_number_concave_planes = m_cutting_planes.size();

    // now collect all the other face-planes
    for (auto const f : mesh.faces())
    {
        auto const rep = mesh.faces()[union_find.find(f.idx.value)];

        if (visited[rep])
            continue;

        visited[rep] = true;

        if (m_input_plane[rep].is_valid())
        {
            m_cutting_planes.push_back(m_input_plane[rep]);
            m_face_of_plane.push_back(rep);
        }
    }
}


void KernelPlaneCut::init_input_planes(pm::vertex_attribute<pos_t> const& positions)
{
    //* construct all face planes
    auto const& mesh = positions.mesh();
    m_input_plane = pm::face_attribute<plane_t>(mesh);

    auto const compute_face_plane = [&](int face_index)
    {
        auto const f = mesh.faces()[face_index];
        auto const pts = f.vertices().template to_array<3>(positions);

        using hpos_t = tg::pos<3, typename geometry_t::normal_scalar_t>;
        using distance_t = typename geometry_t::plane_d_t;

        // higher precision needed as the cross product can go up to 64 bit
        auto n = cross(hpos_t(pts[1]) - hpos_t(pts[0]), hpos_t(pts[2]) - hpos_t(pts[0]));

        if (n == (tg::vec<3, typename geometry_t::normal_scalar_t>::zero))
        {
            m_input_plane[f] = {0, 0, 0, 0};
            return;
        }

        if (m_options.use_unordered_set)
        {
            auto const factor = tg::gcd(tg::gcd(tg::abs(n.x), tg::abs(n.y)), tg::abs(n.z));

            if (factor > 1)
                n /= factor;
        }

        // these assertions only work as long as the normal is less than 64 bit (for now)
        CC_ASSERT(tg::abs(n.x) <= (tg::i64(1) << geometry_t::bits_normal));
        CC_ASSERT(tg::abs(n.y) <= (tg::i64(1) << geometry_t::bits_normal));
        CC_ASSERT(tg::abs(n.z) <= (tg::i64(1) << geometry_t::bits_normal));

        // -dot(n, p0);
        auto const d = ipg::mul<8 * sizeof(distance_t)>(-n.x, pts[0].x) + //
                       ipg::mul<8 * sizeof(distance_t)>(-n.y, pts[0].y) + //
                       ipg::mul<8 * sizeof(distance_t)>(-n.z, pts[0].z);

        m_input_plane[f] = {n.x, n.y, n.z, d};
    };

    auto const n_faces = mesh.faces().size();
#if defined(MK_TBB_ENABLED)
    if (n_faces > m_options.min_faces_for_parallel_setup)
    {
        tbb::parallel_for(tbb::blocked_range<int>(0, n_faces),
                          [&](tbb::blocked_range<int> const& range)
                          {
                              for (int i = range.begin(); i < range.end(); ++i)
                              {
                                  compute_face_plane(i);
                              }
                          });
    }
    else
#endif
    {
        for (int i = 0; i < n_faces; ++i)
        {
            compute_face_plane(i);
        }
    }
}


void KernelPlaneCut::init_edge_state(pm::vertex_attribute<pos_t> const& positions)
{
    auto const& mesh = positions.mesh();
    m_input_edge_state = pm::edge_attribute<edge_state>(mesh);

    auto const compute_edge_classification = [&](int edge_index)
    {
        auto const e = mesh.edges()[edge_index];

        if (e.is_boundary())
        {
            m_input_edge_state[e] = edge_state::boundary;
            return;
        }

        auto const& pa = m_input_plane[e.halfedgeA().face()];
        auto const& pb = m_input_plane[e.halfedgeB().face()];
        if (pa.is_valid() && pb.is_valid())
        {
            auto const v_opp = e.halfedgeB().next().vertex_to();
            auto const pt = positions[v_opp];
            auto const c = ipg::classify(pt, pa);
            switch (c)
            {
            case -1:
                m_input_edge_state[e] = edge_state::convex;
                break;
            case 0:
            { // check orientation
                static constexpr int bits_normal = geometry_t::bits_normal;
                static constexpr int bits_dot = 2 * bits_normal + +2;

                auto const normal_a = pa.normal();
                auto const normal_b = pb.normal();

                auto const dot = ipg::mul<bits_dot>(normal_a.x, normal_b.x) + //
                                 ipg::mul<bits_dot>(normal_a.y, normal_b.y) + //
                                 ipg::mul<bits_dot>(normal_a.z, normal_b.z);

                auto orient = tg::sign(dot);
                if (orient == 1)
                {
                    m_input_edge_state[e] = edge_state::planar;
                }
                else
                {
                    m_input_edge_state[e] = edge_state::concave;
                }
            }
            break;
            case 1:
                m_input_edge_state[e] = edge_state::concave;
                break;
            default:
                break;
            }
        }
        else
        {
            m_input_edge_state[e] = edge_state::degenerate;
        }
    };

    // -1 convex, 0 coplanar, 1 concave, -2 boundary, -3 degenerate
    auto const n_edges = mesh.edges().size();
#if defined(MK_TBB_ENABLED)
    if (m_options.min_faces_for_parallel_setup < n_edges)
    {
        tbb::parallel_for(tbb::blocked_range<int>(0, n_edges),
                          [&](tbb::blocked_range<int> const& range)
                          {
                              for (int i = range.begin(); i < range.end(); ++i)
                              {
                                  compute_edge_classification(i);
                              }
                          });
    }
    else
#endif
    {
        for (int i = 0; i < n_edges; ++i)
        {
            compute_edge_classification(i);
        }
    }

    m_input_is_convex = mesh.edges().all([&](pm::edge_handle e)
                                         { return m_input_edge_state[e] == edge_state::convex || m_input_edge_state[e] == edge_state::planar; });
}


void KernelPlaneCut::init_cutting_planes_uset(pm::vertex_attribute<pos_t> const& positions)
{
    // TRACE();
    m_cutting_planes.clear();
    m_face_of_plane.clear();
    auto& mesh = positions.mesh();
    CC_ASSERT(mesh.is_compact());

    cc::vector<pm::face_handle> concave_faces;
    cc::vector<pm::face_handle> convex_faces;

    cc::set<plane_t> planes;

    for (auto const f : mesh.faces())
    {
        auto const p = m_input_plane(f);

        if (!p.is_valid())
            continue;

        if (planes.contains(p))
            continue;

        planes.add(p);

        if (f.edges().any(
                [&](pm::edge_handle e)
                {
                    return m_input_edge_state[e] == edge_state::concave ||  //
                           m_input_edge_state[e] == edge_state::boundary || //
                           m_input_edge_state[e] == edge_state::degenerate;
                }))
        {
            concave_faces.push_back(f);
        }
        else
        {
            convex_faces.push_back(f);
        }
    }

    //* add concave planes first
    for (auto const f : concave_faces)
    {
        m_cutting_planes.push_back(m_input_plane[f]);
        m_face_of_plane.push_back(f);
    }

    m_number_concave_planes = m_cutting_planes.size();

    for (auto const f : convex_faces)
    {
        m_cutting_planes.push_back(m_input_plane[f]);
        m_face_of_plane.push_back(f);
    }
}

bool KernelPlaneCut::is_infeasible()
{
    if (!m_options.parallel_exact_lp)
        return false;

    if (m_has_queried_future)
        return m_is_infeasible;

    if (m_exact_seidel_solver_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto res = m_exact_seidel_solver_result.get();
        if (res == ExactSeidelSolverPoint::state::infeasible)
        {
            LOGD(Default, Debug, "Finished Seidel Solver before all planes are processed");
            m_is_infeasible = true;
        }
        m_has_queried_future = true; // don't query the future twice!
    }
    return m_is_infeasible;
}


typename KernelPlaneCut::plane_t KernelPlaneCut::face_to_plane(pm::face_handle const& face_handle, pm::vertex_attribute<pos_t> const& positions)
{
    //* pm::face_area loops over vertices and calcs a cross product on every iteration
    //* this can exceed max bits
    auto const vertices = face_handle.vertices().to_vector();
    if (TG_UNLIKELY(vertices.size() < 3))
        return {0, 0, 0, 0};

    auto const p0 = positions(vertices[0]);
    auto const p1 = positions(vertices[1]);
    auto const p2 = positions(vertices[2]);

    return plane_t::from_points_no_gcd(p0, p1, p2);
}

/**
 * @brief converts the mesh into a cube fitting the aabb.
 *
 * This function resets the given mesh and adds a cube fitting the axis-aligned bounding box (AABB) of the initial positions.
 *
 * @tparam pos_t The type representing the position of a vertex.
 * @param mesh The mesh to add the cube.
 * @param int_positions The vertex attribute containing the positions to be scaled.
 */
void KernelPlaneCut::init_with_aabb(pm::vertex_attribute<pos_t> const& input_position, pm::Mesh& mesh, pm::vertex_attribute<pos_t>& output_position)
{
    mesh.clear();
    auto const aabb = tg::aabb_of(input_position);
    auto const size = tg::size_of(aabb);
    pm::objects::add_cube(mesh,
                          [&](pm::vertex_handle v, int x, int y, int z)
                          {
                              output_position[v] = {
                                  aabb.min.x + x * size.width,  //
                                  aabb.min.y + y * size.height, //
                                  aabb.min.z + z * size.depth,  //

                              };
                          });
}


void KernelPlaneCut::set_edge_lines(pm::vertex_attribute<pos_t> const& positions)
{
    for (auto const e : m_mesh.edges())
    {
        //* taken from segment.hh

        auto const& p0 = positions(e.vertexA());
        auto const& p1 = positions(e.vertexB());

        plane_t planeA;
        plane_t planeB;

        auto const d = tg::i64vec3(p1 - p0);
        if (d.x != 0)
        {
            planeA = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(0, 1, 0)));
            planeB = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(0, 0, 1)));
        }
        else if (d.y != 0)
        {
            planeA = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(0, 0, 1)));
            planeB = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(1, 0, 0)));
        }
        else
        {
            planeA = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(1, 0, 0)));
            planeB = plane_t::from_pos_normal(p0, cross(d, tg::i64vec3(0, 1, 0)));
        }

        m_edge_lines(e) = ipg::intersect(planeA, planeB);
    }
}


void KernelPlaneCut::init_supporting_structure(pm::vertex_attribute<pos_t> const& position)
{
    // TRACE();
    m_mesh.clear();

    //* start with aabb cube of mesh
    init_with_aabb(position, m_mesh, m_initial_position);

    init_point4_position(m_initial_position);
    set_edge_lines(m_initial_position);

    if (m_options.use_bb_culling)
        initialize_bounding_volume();

    CC_ASSERT(m_mesh.is_compact());

    auto const compute_face_plane = [&](int face_index)
    {
        auto const f = m_mesh.faces()[face_index];
        auto const pts = f.vertices().to_array<3>(m_initial_position);
        m_supporting_plane[f] = plane_t::from_points_no_gcd(pts[0], pts[1], pts[2]);
        CC_ASSERT(m_supporting_plane[f].is_valid());
    };

    auto const n_faces = m_mesh.faces().size();
#if defined(MK_TBB_ENABLED)
    if (n_faces > m_options.min_faces_for_parallel_setup)
    {
        tbb::parallel_for(tbb::blocked_range<int>(0, n_faces),
                          [&](tbb::blocked_range<int> const& range)
                          {
                              for (int i = range.begin(); i < range.end(); ++i)
                              {
                                  compute_face_plane(i);
                              }
                          });
    }
    else
#endif
    {
        for (int i = 0; i < n_faces; ++i)
        {
            compute_face_plane(i);
        }
    }
}


tg::i8 KernelPlaneCut::classify(pm::vertex_handle const& vertex_handle, plane_t const& plane)
{
    return ipg::classify(m_position_point4(vertex_handle), plane);
}


tg::dpos3 KernelPlaneCut::to_dpos(pm::vertex_handle const& vertex_handle) { return ipg::to_dpos3_fast(m_position_point4(vertex_handle)); }


void KernelPlaneCut::initialize_bounding_volume()
{
    switch (m_options.kdop_k)
    {
    case 3:
        m_3dop.initialize_from_positions(m_initial_position);
        break;
    case 8:
        m_8dop.initialize_from_positions(m_position_dpos);
        break;
    case 9:
        m_9dop.initialize_from_positions(m_position_dpos);
        break;
    case 12:
        m_12dop.initialize_from_positions(m_position_dpos);
        break;
    default:
        CC_UNREACHABLE("invalid kdop_k");
    }
}


void KernelPlaneCut::update_bounding_volume()
{
    // TRACE();
    switch (m_options.kdop_k)
    {
    case 3:
        m_3dop.update(m_c0_vertices, m_position_dpos);
        break;
    case 8:
        m_8dop.update(m_c0_vertices, m_position_dpos);
        break;
    case 9:
        m_9dop.update(m_c0_vertices, m_position_dpos);
        break;
    case 12:
        m_12dop.update(m_c0_vertices, m_position_dpos);
        break;
    default:
        CC_UNREACHABLE("invalid kdop_k");
    }
}


bool KernelPlaneCut::intersects_bounding_volume()
{
    // TRACE();

    switch (m_options.kdop_k)
    {
    case 3:
    {
        return ipg::classify(m_3dop.aabb, m_cutting_plane) >= 0;
    }
    case 8:
    {
        return intersects_bounding_volume(m_8dop);
    }
    case 9:
    {
        return intersects_bounding_volume(m_9dop);
    }
    case 12:
    {
        return intersects_bounding_volume(m_12dop);
    }
    default:
        CC_UNREACHABLE("invalid kdop_k");
    }
}


template <class kdop_t>
bool KernelPlaneCut::intersects_bounding_volume(kdop_t const& kdop)
{
    auto const& axis = kdop.axis;

    //* helper lambdas
    auto dot = [&](auto axis) -> auto { return tg::dot(axis, m_cutting_plane.to_dplane().normal); };
    auto to_ipg_plane = [&](auto kdop, auto idx, auto is_neg) -> plane_t
    {
        auto axis = kdop.axis[idx];
        plane_t result;
        result.a = is_neg ? -axis.x : axis.x;
        result.b = is_neg ? -axis.y : axis.y;
        result.c = is_neg ? -axis.z : axis.z;
        result.d = is_neg ? tg::floor(kdop.distance_min[idx]) : tg::ceil(kdop.distance_max[idx]);

        return result;
    };

    //* find axis that maximizes abs(dot)
    auto max_dot = tg::abs(dot(axis[0]));
    size_t max_idx = 0;
    for (size_t i = 1; i < kdop.size(); i++)
    {
        auto const curr_dot = tg::abs(dot(axis[i]));
        if (curr_dot <= max_dot)
            continue;

        max_dot = curr_dot;
        max_idx = i;
    }

    //* find orientation
    auto const is_neg = (dot(axis[max_idx]) > dot(-axis[max_idx])) ? false : true;
    plane_t max_dot_plane = to_ipg_plane(kdop, max_idx, is_neg);

    cc::vector<plane_t> planes;
    //* construct planes
    for (size_t i = 0; i < kdop.size() - 1; i++)
    {
        if (i == max_idx)
            continue;
        planes.emplace_back(to_ipg_plane(kdop, i, true));
        planes.emplace_back(to_ipg_plane(kdop, i, false));
    }
    //* construct relevant kdop corner candidates
    cc::vector<point4_t> corner_candidates;
    for (auto const plane_a : planes)
    {
        for (auto const plane_b : planes)
        {
            corner_candidates.emplace_back(ipg::intersect(plane_a, plane_b, max_dot_plane));
        }
    }

    //* find real corners
    cc::vector<point4_t> real_corners; // very few so probably faster to use new vector instead of deleting from corner_candidates?
    bool all_behind = true;
    for (auto const& candidate : corner_candidates)
    {
        all_behind = true;

        for (auto const plane : planes)
        {
            if (ipg::classify(candidate, plane) <= 0)
                continue;

            all_behind = false;
            break;
        }
        if (all_behind)
            real_corners.push_back(candidate);
    }

    //* check if all real corners are behind the cutting plane > no intersection

    for (auto const& corner : real_corners)
    {
        if (ipg::classify(corner, m_cutting_plane) >= 0)
            return true;
    }
    return false;
}


tg::pos3 KernelPlaneCut::to_pos(pm::vertex_handle const& vertex_handle) { return ipg::to_pos3_fast(m_position_point4(vertex_handle)); }

//* we assume not many vertices are within double epsilon of the cutting plane
//* this only checks all N1 neighbors for a sign change and returns invalid if none intersect the cutting plane

pm::halfedge_handle KernelPlaneCut::edge_descent_exact(pm::vertex_handle const& vertex)
{
    for (auto const halfedge : vertex.outgoing_halfedges())
    {
        //* return if signs are different or both are on the cutting plane
        auto const c0 = classify(halfedge.vertex_from(), m_cutting_plane);
        auto const c1 = classify(halfedge.vertex_to(), m_cutting_plane);

        if CC_CONDITION_UNLIKELY (c0 == 0)
        {
            m_c0_vertex = halfedge.vertex_from();
            return halfedge.opposite();
        }
        if CC_CONDITION_UNLIKELY (c1 == 0)
        {
            m_c0_vertex = halfedge.vertex_to();
            return halfedge;
        }
        if (c0 != c1)
        {
            return halfedge;
        }
    }
    return pm::halfedge_handle::invalid;
}

//* returns invalid handle if no intersecting halfedge is found

pm::halfedge_handle KernelPlaneCut::edge_descent(pm::vertex_handle const& start_vertex)
{
    // TRACE();
    if (classify(start_vertex, m_cutting_plane) == 0)
    {
        m_c0_vertex = start_vertex;
        m_is_c0_vertex[start_vertex] = true;
    }

    auto closest_vertex = start_vertex;

    auto const plane = m_cutting_plane.to_dplane();
    bool found_closer_vertex = true;

    auto min_distance = tg::signed_distance(m_position_dpos(closest_vertex), plane);

    auto epsilon = precision_for(min_distance);

    while (found_closer_vertex)
    {
        found_closer_vertex = false;

        //* find halfedge with smallest distance to cutting plane
        for (auto const neighbor_he : closest_vertex.outgoing_halfedges())
        {
            auto const neighbor = neighbor_he.vertex_to();
            auto const current_distance = tg::signed_distance(m_position_dpos(neighbor), plane);
            auto const current_eps = precision_for(current_distance);
            if (current_eps > epsilon)
                epsilon = current_eps;

            if (TG_UNLIKELY(tg::abs(current_distance) < epsilon))
                return edge_descent_exact(neighbor);
            if (TG_UNLIKELY(tg::sign(current_distance) != tg::sign(min_distance)))
                return neighbor_he;
            if (TG_LIKELY(tg::abs(current_distance) >= tg::abs(min_distance)))
                continue;

            if (TG_UNLIKELY(tg::abs(current_distance) < epsilon))
                edge_descent_exact(neighbor);

            closest_vertex = neighbor;
            min_distance = current_distance;

            //* as long as there exists a neighbor closer to the cutting plane we assume the plane intersects the mesh
            found_closer_vertex = true;
        }
    }

    //* if no intersection is found we check again with exact computation
    return edge_descent_exact(closest_vertex);
}

void KernelPlaneCut::show_current_state(gv::canvas_data& canvas_data)
{
    // return;

    pm::vertex_attribute<tg::dpos3> pos(m_mesh);
    for (auto const vertex_handle : m_mesh.vertices())
    {
        if (m_position_point4(vertex_handle).is_valid())
            pos(vertex_handle) = ipg::to_dpos3_fast(m_position_point4(vertex_handle));
        else
            pos(vertex_handle) = tg::dpos3(0, 0, 0);
    }

    add_plane(canvas_data, m_cutting_plane);

    auto v = gv::view();
    v.configure(gv::print_mode, gv::no_grid);

    auto canvas = gv::canvas();
    canvas.add_data(canvas_data);

    auto const dplane = m_cutting_plane.to_dplane();
    auto const aabb = tg::aabb_of(pos);
    auto const diag = tg::distance(aabb.min, aabb.max);
    auto const center = tg::dpos3(dplane.normal * dplane.dis);

    canvas.add_line(tg::dpos3(center), center + dplane.normal * diag * 0.1).color(tg::color3::red);

    for (auto const vertex_handle : m_mesh.vertices())
    {
        auto const point = pos(vertex_handle);
        canvas.add_point(point);
    }

    canvas.add_lines(pos);
    canvas.add_faces(pos);

    // input mesh
    canvas.add_lines(m_input_pos).color(tg::color3::cyan);
    canvas.add_points(m_input_pos).color(tg::color3::cyan);
}

//* returns true if at least one c1 vertex was deleted

bool KernelPlaneCut::delete_c1_vertices()
{
    if (m_c0_vertex.is_invalid())
        return false;

    // TRACE();
    // CC_ASSERT(m_mesh.is_compact());
    pm::vertex_handle initial_c1_vertex = pm::vertex_handle::invalid;
    for (auto neighbor : m_c0_vertex.adjacent_vertices())
    {
        auto const c = classify(neighbor, m_cutting_plane);
        if (c == 1)
        {
            initial_c1_vertex = neighbor;
            break;
        }
    }
    if (initial_c1_vertex.is_invalid())
        return false;

    cc::vector<pm::vertex_handle> stack;
    stack.push_back(initial_c1_vertex);
    m_visited_c1_vertex[initial_c1_vertex] = true;

    while (!stack.empty())
    {
        auto const current_vertex = stack.get_and_pop_back();

        for (auto neighbor : current_vertex.adjacent_vertices())
        {
            if (m_is_c0_vertex[neighbor] || m_visited_c1_vertex[neighbor])
                continue;

            stack.push_back(neighbor);
            m_visited_c1_vertex[neighbor] = true;
        }
        CC_ASSERT(classify(current_vertex, m_cutting_plane) == 1);
        m_mesh.vertices().remove(current_vertex);
    }

    return true;
}


void KernelPlaneCut::fill_cut_hole()
{
    // TRACE();
    if (m_mesh.vertices().size() < 3 || m_c0_vertices.size() < 3) // no face to fill
        return;

    auto first_halfedge = m_mesh.halfedges().add_or_get(m_c0_vertices[0], m_c0_vertices[1]);
    CC_ASSERT(first_halfedge.is_boundary() || first_halfedge.opposite().is_boundary());

    first_halfedge = first_halfedge.is_boundary() ? first_halfedge : first_halfedge.opposite();

    auto const new_face = m_mesh.faces().fill(first_halfedge);
    m_supporting_plane[new_face] = m_cutting_plane;
    m_input_face[new_face] = m_cutting_plane_original_face;
}


void KernelPlaneCut::split_halfedge(pm::halfedge_handle const& halfedge)
{
    auto const current_line = m_edge_lines(halfedge.edge());

    auto const intersection_point = ipg::intersect(current_line, m_cutting_plane);

    auto const new_vertex_handle = m_mesh.halfedges().split(halfedge);
    m_position_point4(new_vertex_handle) = intersection_point;
    m_position_dpos(new_vertex_handle) = to_dpos(new_vertex_handle);

    auto const new_edge = halfedge.next().edge();
    m_edge_lines(new_edge) = {current_line};
}


void KernelPlaneCut::split_face(pm::vertex_handle vertex_from, pm::vertex_handle vertex_to, pm::face_handle face)
{
    //* if cut us 2d we can get an invalid face because the mesh is no longer closed
    if (face.is_invalid())
        return;

    auto const h_from = vertex_from.incoming_halfedges().where([&](auto const h) { return h.face() == face; }).first();
    auto const h_to = vertex_to.incoming_halfedges().where([&](auto const h) { return h.face() == face; }).first();

    auto const h_new = m_mesh.faces().cut(face, h_from, h_to);
    auto const f_new = h_new.opposite_face();
    CC_ASSERT(face == h_new.face());

    m_edge_lines[h_new] = ipg::intersect(m_cutting_plane, m_supporting_plane[face]);

    m_supporting_plane[f_new] = m_supporting_plane[face];
    m_input_face[f_new] = m_input_face[face];
}


bool KernelPlaneCut::signs_different(pm::vertex_handle const& vA, pm::vertex_handle const& vB)
{
    auto const pointA = m_position_point4(vA);
    auto const pointB = m_position_point4(vB);
    auto const cA = ipg::classify(pointA, m_cutting_plane);
    auto const cB = ipg::classify(pointB, m_cutting_plane);

    return tg::sign(cA) != tg::sign(cB);
}


bool KernelPlaneCut::signs_different(pm::edge_handle const& edge) { return signs_different(edge.vertexA(), edge.vertexB()); }


bool KernelPlaneCut::signs_different(pm::halfedge_handle const& halfedge) { return signs_different(halfedge.vertex_to(), halfedge.vertex_from()); }

//* returns invalid handle if no intersecting face is found

pm::halfedge_handle KernelPlaneCut::skip_non_intersecting_faces(pm::halfedge_handle current_halfedge)
{
    auto const current_c0_vertex = current_halfedge.vertex_to();
    auto prev_halfedge = current_halfedge;

    for (int i = 0; i < current_c0_vertex.faces().size(); i++)
    {
        current_halfedge = prev_halfedge.opposite();
        prev_halfedge = current_halfedge.prev();

        if (signs_different(current_halfedge.vertex_to(), prev_halfedge.vertex_from()))
            return current_halfedge;
    }
    //* return invalid after a full loop no intersecting face is found
    return pm::halfedge_handle::invalid;
}


void KernelPlaneCut::marching(pm::halfedge_handle const& start_halfedge)
{
    CC_ASSERT(classify(start_halfedge.vertex_to(), m_cutting_plane) == 0
              || classify(start_halfedge.vertex_from(), m_cutting_plane) != classify(start_halfedge.vertex_to(), m_cutting_plane));

    // TRACE();
    auto current_halfedge = start_halfedge;

    pm::vertex_handle current_c0_vertex = pm::vertex_handle::invalid;
    pm::vertex_handle prev_c0_vertex = pm::vertex_handle::invalid;

    //* march along the cutting plane placing c0 vertices on intersections
    do
    {
        LOGD(Default, Trace, "current halfedge %s;  start_halfedge %s", current_halfedge.idx.value, start_halfedge.idx.value);
        auto pointA = m_position_point4(current_halfedge.vertex_from());
        auto pointB = m_position_point4(current_halfedge.vertex_to());
        auto cA = ipg::classify(pointA, m_cutting_plane);
        auto cB = ipg::classify(pointB, m_cutting_plane);

        //* keep tracing if no sign change
        auto const first_he = current_halfedge;
        while (!signs_different(current_halfedge) || cA == 0)
        {
            current_halfedge = current_halfedge.next();

            pointA = m_position_point4(current_halfedge.vertex_from());
            pointB = m_position_point4(current_halfedge.vertex_to());
            cA = ipg::classify(pointA, m_cutting_plane);
            cB = ipg::classify(pointB, m_cutting_plane);

            if (cA == 0)
            {
                m_is_c0_vertex[current_halfedge.vertex_from()] = true;
                m_c0_vertices.push_back(current_halfedge.vertex_from());
            }

            //* break if we have a full loop
            if (current_halfedge == first_he)
                return; // changed this from break to return, hope this doesnt break anything, but i had an infinite loop here with 314438.obj
        }

        //* if on different sides of the plane split halfedge
        if (cA * cB == -1)
            split_halfedge(current_halfedge);

        //* now current_halfedge points towards vertex on cutting plane
        prev_c0_vertex = current_c0_vertex;
        current_c0_vertex = current_halfedge.vertex_to();

        //* check if we had a full loop without finding other vertex
        if (prev_c0_vertex == current_c0_vertex)
            break;

        m_c0_vertices.push_back(current_c0_vertex);
        m_c0_vertex = current_c0_vertex;
        CC_ASSERT(m_c0_vertex.is_valid());
        m_is_c0_vertex[current_c0_vertex] = true;

        //* connect with previous intersecting vertex by splitting face
        if (prev_c0_vertex.is_valid() && !pm::are_adjacent(current_c0_vertex, prev_c0_vertex))
            split_face(current_c0_vertex, prev_c0_vertex, current_halfedge.face());

        //* check for non intersecting faces and skip them
        current_halfedge = skip_non_intersecting_faces(current_halfedge);
        if (current_halfedge == pm::halfedge_handle::invalid)
            break;

    } while (m_c0_vertices.size() < 2 || current_c0_vertex != m_c0_vertices.front());

    // since current_c0_vertex != m_c0_vertices.front() the first one gets added twice
    m_c0_vertices.pop_back();
}

//* cuts the given mesh with the given plane, mesh is modified and a vertex_attribute<ipg::point4> is return containing the new positions

void KernelPlaneCut::compute_mesh_kernel()
{
    // TRACE();
    LOGD(Default, Debug, "cutting plane size %s", m_cutting_planes.size());

    TRACE("cutting-all-planes");
    TRACE_BEGIN("cutting-concave-planes");
    auto trace_finished = false;

    for (size_t i = 0; i < m_cutting_planes.size(); i++)
    {
        if (is_infeasible())
        {
            m_benchmark_data.lp_early_out = true;
            m_has_kernel = false;
            return;
        }

        if (i == m_number_concave_planes)
        {
            TRACE_END();
            trace_finished = true;
        }

        m_cutting_plane = m_cutting_planes[i];
        m_cutting_plane_original_face = m_face_of_plane[i];

        if (m_options.use_bb_culling && /*i > m_number_concave_planes &&*/ !intersects_bounding_volume())
            continue;

        LOGD(Default, Debug, "cutting plane %s/%s", i, m_cutting_planes.size());

        //* find halfedge that gets intersected by cutting plane
        auto const start_vertex = m_mesh.vertices().last();
        auto start_halfedge = edge_descent(start_vertex);
        // auto start_halfedge = edge_descent_old();
        if (start_halfedge == pm::halfedge_handle::invalid) // no halfedge crossing the boundary
        {
            if (classify(start_vertex, m_cutting_plane) < 0)
                continue; // entire poly inside

            if (!m_c0_vertex.is_valid())
            {
                //* if the plane does not intersect but the vertex is on the positive site the kernel is empty
                m_has_kernel = false;
                return;
            }
        }
        else
        {
            marching(start_halfedge);
        }

        auto const proper_cut = delete_c1_vertices();

        if (proper_cut)
            fill_cut_hole();

        if (m_options.use_bb_culling && proper_cut /*&& i > m_number_concave_planes*/)
            update_bounding_volume();

        m_is_c0_vertex.clear();
        m_c0_vertices.clear();
        m_visited_c1_vertex.clear();
        m_c0_vertex = pm::vertex_handle::invalid;
    }
    if (!trace_finished)
        TRACE_END();

    m_exact_seidel_solver.stop(); // cancel the LP solver if still running

    LOGD(Default, Info, "compute mesh kernel done!");

    if (m_mesh.vertices().size() != 0)
        m_has_kernel = true;

    if (m_debug)
        m_mesh.assert_consistency();

    LOGD(Default, Debug, "done!");
}


void KernelPlaneCut::add_plane(gv::canvas_data& canvas, plane_t const& plane, tg::color4 const& color)
{
    auto const& dplane = plane.to_dplane();
    auto const aabb = tg::aabb_of(m_initial_position);
    auto length = tg::length(aabb.max - aabb.min);

    auto const plane_origin = tg::dpos3(dplane.normal * dplane.dis);

    tg::dvec3 vec1;

    if (dplane.normal.z != 0)
    {
        vec1 = tg::normalize_safe(tg::dvec3(1, 0, dplane.normal.x / dplane.normal.z));
    }
    else if (dplane.normal.y != 0)
    {
        vec1 = tg::normalize_safe(tg::dvec3(1, dplane.normal.x / dplane.normal.y, 0));
    }
    else
    {
        vec1 = tg::normalize_safe(tg::dvec3(1, 0, 0));
    }

    tg::dvec3 const vec2 = tg::normalize_safe(tg::cross(dplane.normal, vec1));
    vec1 = tg::normalize_safe(tg::cross(dplane.normal, vec2));

    CC_ASSERT(tg::abs(tg::dot(vec1, vec2)) < 0.0001);

    auto const top_right = plane_origin + vec1 * length / 2 + vec2 * length / 2;
    auto const top_left = top_right - vec1 * length;
    auto const bottom_left = top_left - vec2 * length;
    auto const bottom_right = top_right - vec2 * length;

    canvas.add_face(top_right, top_left, bottom_left, bottom_right, gv::material(color));
}

} // namespace mk
