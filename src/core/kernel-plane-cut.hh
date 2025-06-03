#pragma once

#include <future>

// system
#include <clean-core/hash.hh>
#include <clean-core/pair.hh>
#include <clean-core/vector.hh>

// extern
#include <polymesh/Mesh.hh>
#include <polymesh/attributes/fast_clear_attribute.hh>

#include <integer-plane-geometry/geometry.hh>
#include <integer-plane-geometry/integer_math.hh>
#include <integer-plane-geometry/plane.hh>

#include <polymesh/Mesh.hh>
#include <typed-geometry/types/scalars/fixed_int.hh>

#include <glow-extras/viewer/canvas.hh>

// internal
#include <core/ExactSeidelSolverPoint.hh>
#include <core/benchmark_data.hh>
#include <core/kdop.hh>
#include <core/options.hh>

namespace mk
{
class KernelPlaneCut
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;
    using vec_t = typename geometry_t::vec_t;
    using point4_t = typename geometry_t::point4_t;
    using plane_t = typename geometry_t::plane_t;
    using line_t = typename ipg::line<geometry_t>;

public: // API
    KernelPlaneCut() = default;

    KernelPlaneCut(pm::vertex_attribute<pos_t> const& input_positions, kernel_options const& options = {});

    void compute_kernel(pm::vertex_attribute<pos_t> const& input_positions, kernel_options const& options = {});

    bool has_kernel() const { return m_has_kernel; }

    bool input_is_convex() const { return m_input_is_convex; }

    pm::Mesh const& mesh() const { return m_mesh; }

    pm::vertex_attribute<point4_t> const& position_point4() const { return m_position_point4; }

    mk::benchmark_data const& stats() const { return m_benchmark_data; }

private: // member
    /// settings
    kernel_options m_options;

    /// planes of the input mesh
    pm::face_attribute<plane_t> m_input_plane;

    enum class edge_state
    {
        unclassified,
        convex,
        planar,
        concave,
        boundary,
        degenerate,
    };
    pm::edge_attribute<edge_state> m_input_edge_state;

    /// cutting planes
    cc::vector<plane_t> m_cutting_planes;
    cc::vector<pm::face_handle> m_face_of_plane;
    cc::vector<bool> m_plane_handeled;
    size_t m_number_concave_planes = 0;

    //* runtime specific

    /// current cutting plane
    plane_t m_cutting_plane;
    /// face of the input plane generating the cutting plane
    pm::face_handle m_cutting_plane_original_face;
    k_dop<3, int> m_3dop; // aabb
    k_dop<8, double> m_8dop;
    k_dop<9, double> m_9dop;
    k_dop<12, double> m_12dop;
    cc::vector<pm::vertex_handle> m_c0_vertices;

    /// kernel mesh
    pm::Mesh m_mesh;
    /// initial positions
    pm::vertex_attribute<pos_t> m_initial_position{m_mesh};
    /// homogeneous exact coords
    pm::vertex_attribute<point4_t> m_position_point4{m_mesh};
    /// rounded double coords for output
    pm::vertex_attribute<tg::dpos3> m_position_dpos{m_mesh};
    /// exact representation of edge line
    pm::edge_attribute<line_t> m_edge_lines{m_mesh};
    /// supporting planes of each triangle
    pm::face_attribute<plane_t> m_supporting_plane{m_mesh};
    /// maps each face to a generating input face
    pm::face_attribute<pm::face_handle> m_input_face{m_mesh};
    /// fast clear for c1 vertices
    pm::fast_clear_attribute<bool, pm::vertex_tag> m_is_c0_vertex = pm::make_fast_clear_attribute(m_mesh.vertices(), false);
    pm::fast_clear_attribute<bool, pm::vertex_tag> m_visited_c1_vertex = pm::make_fast_clear_attribute(m_mesh.vertices(), false);
    pm::vertex_handle m_c0_vertex;

    /// exact seidel solver for early out check
    ExactSeidelSolverPoint m_exact_seidel_solver;
    std::future<ExactSeidelSolverPoint::state> m_exact_seidel_solver_result;
    bool m_has_queried_future = false; // avoid query more than once
    bool m_is_infeasible = false;

    bool m_has_kernel = false;
    std::atomic<bool> m_input_is_convex = true;

    benchmark_data m_benchmark_data;

    //* debug only
    bool m_debug = false;
    pm::vertex_attribute<pos_t> m_input_pos;

private: // helper
    void reset();
    bool has_trivial_solution();
    void init_point4_position(pm::vertex_attribute<pos_t> const& positions);
    void init_cutting_planes_flood_fill(pm::vertex_attribute<pos_t> const& positions);
    void init_input_planes(pm::vertex_attribute<pos_t> const& positions);
    void init_edge_state(pm::vertex_attribute<pos_t> const& positions);
    void init_with_aabb(pm::vertex_attribute<pos_t> const& input_position, pm::Mesh& mesh, pm::vertex_attribute<pos_t>& output_position);
    void classify_vertices(plane_t const& cutting_plane);
    void init_cutting_planes_uset(pm::vertex_attribute<pos_t> const& positions);

    /// returns true, if the exact seidel solver has finished and determided that the kernel is empty
    bool is_infeasible();

    void compute_mesh_kernel();
    bool is_convex();
    bool kernel_is_empty();
    void set_edge_lines(pm::vertex_attribute<pos_t> const& positions);
    void init_supporting_structure(pm::vertex_attribute<pos_t> const& position);

    pm::halfedge_handle edge_descent(pm::vertex_handle const& start_vertex);
    pm::halfedge_handle edge_descent_exact(pm::vertex_handle const& vertex);
    void marching(pm::halfedge_handle const& start_halfedge);
    bool delete_c1_vertices();
    void fill_cut_hole();

    void split_halfedge(pm::halfedge_handle const& halfedge);
    void split_face(polymesh::vertex_handle vertex_from, polymesh::vertex_handle vertex_to, polymesh::face_handle face);
    pm::halfedge_handle skip_non_intersecting_faces(pm::halfedge_handle current_halfedge);

    bool signs_different(pm::edge_handle const& edge);
    bool signs_different(pm::vertex_handle const& vA, pm::vertex_handle const& vB);
    bool signs_different(pm::halfedge_handle const& halfedge);
    tg::i8 classify(pm::vertex_handle const& vertex_handle, plane_t const& plane);
    tg::dpos3 to_dpos(pm::vertex_handle const& vertex_handle);
    tg::pos3 to_pos(pm::vertex_handle const& vertex_handle);
    plane_t face_to_plane(pm::face_handle const& face_handle, pm::vertex_attribute<pos_t> const& positions);

    void initialize_bounding_volume();
    void update_bounding_volume();

    bool intersects_bounding_volume();

    template <class kdop_t>
    bool intersects_bounding_volume(kdop_t const& kdop);

    //* debug only
    void add_plane(gv::canvas_data& canvas, plane_t const& plane, tg::color4 const& color = tg::color4(0, 1, 0, 0.5));
    void show_current_state(gv::canvas_data& canvas_data);
};

} // namespace mk
