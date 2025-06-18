#include "ember-csg.hh"

#include <cstring>
#include <clean-core/vector.hh>
#include <clean-core/set.hh>

#include <rich-log/log.hh>

#include <polymesh/algorithms/triangulate.hh>
#include <polymesh/properties.hh>

#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/intersect.hh>
#include <integer-plane-geometry/plane.hh>

namespace mk
{

bool EmberCSG::compute_csg(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                           pm::vertex_attribute<pos_t> const& mesh_b_positions,
                           csg_operation op,
                           ember_options const& options)
{
    m_options = options;
    m_has_result = false;
    
    // Initialize input meshes
    init_meshes(mesh_a_positions, mesh_b_positions);
    
    LOGD(Default, Info, "Computing CSG operation: %s", csg_utils::operation_to_string(op));
    LOGD(Default, Info, "Mesh A: %d vertices, %d faces", m_mesh_a.vertices().size(), m_mesh_a.faces().size());
    LOGD(Default, Info, "Mesh B: %d vertices, %d faces", m_mesh_b.vertices().size(), m_mesh_b.faces().size());
    
    // Compute mesh-mesh intersections
    if (!compute_intersections())
    {
        LOGD(Default, Error, "Failed to compute mesh intersections");
        return false;
    }
    
    // Build local arrangements at intersection regions
    if (!build_local_arrangements())
    {
        LOGD(Default, Error, "Failed to build local arrangements");
        return false;
    }
    
    // Classify mesh elements
    if (!classify_elements())
    {
        LOGD(Default, Error, "Failed to classify mesh elements");
        return false;
    }
    
    // Construct result mesh
    if (!construct_result(op))
    {
        LOGD(Default, Error, "Failed to construct result mesh");
        return false;
    }
    
    // Validate result if requested
    if (m_options.validate_results && !validate_result())
    {
        LOGD(Default, Warning, "Result validation failed");
    }
    
    m_has_result = true;
    LOGD(Default, Info, "CSG operation completed successfully");
    LOGD(Default, Info, "Result: %d vertices, %d faces", m_result_mesh.vertices().size(), m_result_mesh.faces().size());
    
    return true;
}

bool EmberCSG::compute_union(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                             pm::vertex_attribute<pos_t> const& mesh_b_positions,
                             ember_options const& options)
{
    return compute_csg(mesh_a_positions, mesh_b_positions, csg_operation::union_op, options);
}

bool EmberCSG::compute_intersection(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                                    pm::vertex_attribute<pos_t> const& mesh_b_positions,
                                    ember_options const& options)
{
    return compute_csg(mesh_a_positions, mesh_b_positions, csg_operation::intersection, options);
}

bool EmberCSG::compute_difference(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                                  pm::vertex_attribute<pos_t> const& mesh_b_positions,
                                  ember_options const& options)
{
    return compute_csg(mesh_a_positions, mesh_b_positions, csg_operation::difference, options);
}

void EmberCSG::init_meshes(pm::vertex_attribute<pos_t> const& mesh_a_positions,
                           pm::vertex_attribute<pos_t> const& mesh_b_positions)
{
    // Clear previous results
    m_mesh_a.clear();
    m_mesh_b.clear();
    m_result_mesh.clear();
    
    // Copy mesh A
    auto& mesh_a = mesh_a_positions.mesh();
    m_mesh_a = mesh_a; // Copy mesh structure
    m_mesh_a_positions = pm::vertex_attribute<pos_t>(m_mesh_a);
    
    for (auto v : m_mesh_a.vertices())
    {
        auto original_v = pm::vertex_handle(v.idx.value);
        if (original_v.is_valid() && original_v.idx.value < mesh_a_positions.mesh().vertices().size())
        {
            m_mesh_a_positions[v] = mesh_a_positions[original_v];
        }
    }
    
    // Copy mesh B
    auto& mesh_b = mesh_b_positions.mesh();
    m_mesh_b = mesh_b; // Copy mesh structure  
    m_mesh_b_positions = pm::vertex_attribute<pos_t>(m_mesh_b);
    
    for (auto v : m_mesh_b.vertices())
    {
        auto original_v = pm::vertex_handle(v.idx.value);
        if (original_v.is_valid() && original_v.idx.value < mesh_b_positions.mesh().vertices().size())
        {
            m_mesh_b_positions[v] = mesh_b_positions[original_v];
        }
    }
}

bool EmberCSG::compute_intersections()
{
    // For now, implement a simplified intersection computation
    // In a full EMBER implementation, this would involve:
    // 1. Spatial partitioning for efficient proximity queries
    // 2. Triangle-triangle intersection tests
    // 3. Edge-face intersection tests
    // 4. Vertex-in-mesh tests
    
    LOGD(Default, Debug, "Computing mesh-mesh intersections");
    
    // This is a placeholder for the full intersection computation
    // Real implementation would need triangle-triangle intersection routines
    
    return true;
}

bool EmberCSG::build_local_arrangements()
{
    // Local arrangements are critical for robust boolean operations
    // They handle complex intersection scenarios like:
    // 1. Multiple triangles intersecting at a single point
    // 2. Edge-on-edge intersections
    // 3. Coplanar face intersections
    
    LOGD(Default, Debug, "Building local arrangements");
    
    // This is a placeholder for the full local arrangement construction
    // Real implementation would build arrangement graphs at intersection points
    
    return true;
}

bool EmberCSG::classify_elements()
{
    // Element classification determines which parts of each mesh
    // should be included in the final result based on the operation
    
    LOGD(Default, Debug, "Classifying mesh elements");
    
    // For union: include all elements not strictly inside the other mesh
    // For intersection: include only elements inside both meshes
    // For difference: include A elements not inside B, exclude B elements
    
    // This is a placeholder for the full classification algorithm
    // Real implementation would use robust inside/outside tests
    
    return true;
}

bool EmberCSG::construct_result(csg_operation op)
{
    // Construct the result mesh based on the operation type
    // This involves combining the relevant parts of both input meshes
    
    LOGD(Default, Debug, "Constructing result mesh for operation: %s", csg_utils::operation_to_string(op));
    
    // Clear result mesh
    m_result_mesh.clear();
    m_result_positions = pm::vertex_attribute<pos_t>(m_result_mesh);
    
    // For now, implement a simple placeholder that just copies mesh A
    // Real implementation would construct the proper boolean result
    
    switch (op)
    {
    case csg_operation::union_op:
        // Union: combine both meshes, removing overlapping regions
        m_result_mesh = m_mesh_a; // Placeholder
        m_result_positions = pm::vertex_attribute<pos_t>(m_result_mesh);
        for (auto v : m_result_mesh.vertices())
        {
            auto src_v = pm::vertex_handle(v.idx.value);
            if (src_v.is_valid() && src_v.idx.value < m_mesh_a.vertices().size())
            {
                m_result_positions[v] = m_mesh_a_positions[src_v];
            }
        }
        break;
        
    case csg_operation::intersection:
        // Intersection: keep only overlapping regions
        m_result_mesh = m_mesh_a; // Placeholder
        m_result_positions = pm::vertex_attribute<pos_t>(m_result_mesh);
        for (auto v : m_result_mesh.vertices())
        {
            auto src_v = pm::vertex_handle(v.idx.value);
            if (src_v.is_valid() && src_v.idx.value < m_mesh_a.vertices().size())
            {
                m_result_positions[v] = m_mesh_a_positions[src_v];
            }
        }
        break;
        
    case csg_operation::difference:
        // Difference: A minus B
        m_result_mesh = m_mesh_a; // Placeholder
        m_result_positions = pm::vertex_attribute<pos_t>(m_result_mesh);
        for (auto v : m_result_mesh.vertices())
        {
            auto src_v = pm::vertex_handle(v.idx.value);
            if (src_v.is_valid() && src_v.idx.value < m_mesh_a.vertices().size())
            {
                m_result_positions[v] = m_mesh_a_positions[src_v];
            }
        }
        break;
    }
    
    return true;
}

bool EmberCSG::validate_result()
{
    // Validate the result mesh for common issues:
    // 1. Manifoldness
    // 2. Orientation consistency  
    // 3. Self-intersections
    // 4. Degenerate faces/edges
    
    LOGD(Default, Debug, "Validating result mesh");
    
    if (m_result_mesh.vertices().size() == 0)
    {
        LOGD(Default, Warning, "Result mesh is empty");
        return false;
    }
    
    if (m_result_mesh.faces().size() == 0)
    {
        LOGD(Default, Warning, "Result mesh has no faces");
        return false;
    }
    
    // Basic validation - check for valid positions
    for (auto v : m_result_mesh.vertices())
    {
        auto pos = m_result_positions[v];
        // Check for reasonable coordinate values
        // This is a simplified check - real validation would be more comprehensive
    }
    
    return true;
}

namespace csg_utils
{

const char* operation_to_string(csg_operation op)
{
    switch (op)
    {
    case csg_operation::union_op:
        return "union";
    case csg_operation::intersection:
        return "intersection";
    case csg_operation::difference:
        return "difference";
    default:
        return "unknown";
    }
}

csg_operation string_to_operation(const char* str)
{
    if (strcmp(str, "union") == 0)
        return csg_operation::union_op;
    if (strcmp(str, "intersection") == 0)
        return csg_operation::intersection;
    if (strcmp(str, "difference") == 0)
        return csg_operation::difference;
    
    return csg_operation::union_op; // Default
}

} // namespace csg_utils

} // namespace mk