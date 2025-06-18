#pragma once

#include <polymesh/Mesh.hh>
#include <integer-plane-geometry/geometry.hh>
#include <core/options.hh>

namespace mk
{

/// EMBER CSG operation types
enum class csg_operation
{
    union_op,
    intersection,
    difference
};

/// Options for EMBER CSG operations
struct ember_options
{
    bool use_exact_arithmetic = true;
    bool use_local_arrangements = true;
    bool validate_results = true;
    double tolerance = 1e-10;
};

/// EMBER CSG processor for exact mesh boolean operations
class EmberCSG
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;
    using vec_t = typename geometry_t::vec_t;
    using point4_t = typename geometry_t::point4_t;
    using plane_t = typename geometry_t::plane_t;

public: // API
    EmberCSG() = default;
    
    /// Perform CSG operation between two meshes
    bool compute_csg(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                     pm::vertex_attribute<pos_t> const& mesh_b_positions,
                     csg_operation op,
                     ember_options const& options = {});

    /// Get the result mesh
    pm::Mesh const& result_mesh() const { return m_result_mesh; }
    
    /// Get the result mesh positions
    pm::vertex_attribute<pos_t> const& result_positions() const { return m_result_positions; }
    
    /// Check if result is valid
    bool has_result() const { return m_has_result; }

    /// Perform union operation
    bool compute_union(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                       pm::vertex_attribute<pos_t> const& mesh_b_positions,
                       ember_options const& options = {});

    /// Perform intersection operation  
    bool compute_intersection(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                              pm::vertex_attribute<pos_t> const& mesh_b_positions,
                              ember_options const& options = {});

    /// Perform difference operation (A - B)
    bool compute_difference(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                            pm::vertex_attribute<pos_t> const& mesh_b_positions,
                            ember_options const& options = {});

private: // members
    pm::Mesh m_mesh_a;
    pm::Mesh m_mesh_b;
    pm::Mesh m_result_mesh;
    
    pm::vertex_attribute<pos_t> m_mesh_a_positions{m_mesh_a};
    pm::vertex_attribute<pos_t> m_mesh_b_positions{m_mesh_b};
    pm::vertex_attribute<pos_t> m_result_positions{m_result_mesh};
    
    ember_options m_options;
    bool m_has_result = false;

private: // implementation
    /// Initialize meshes from input
    void init_meshes(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                     pm::vertex_attribute<pos_t> const& mesh_b_positions);

    /// Compute mesh-mesh intersections using local arrangements
    bool compute_intersections();
    
    /// Build local arrangements at intersection regions
    bool build_local_arrangements();
    
    /// Classify mesh elements relative to other mesh
    bool classify_elements();
    
    /// Construct result mesh based on operation type
    bool construct_result(csg_operation op);
    
    /// Validate result mesh for correctness
    bool validate_result();
};

/// Utility functions for CSG operations
namespace csg_utils
{
    /// Convert CSG operation to string
    const char* operation_to_string(csg_operation op);
    
    /// Parse CSG operation from string
    csg_operation string_to_operation(const char* str);
}

} // namespace mk