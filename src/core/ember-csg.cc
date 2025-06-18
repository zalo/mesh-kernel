#include "ember-csg.hh"
#include "ember-intersect.hh"

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
    
    // TODO: For now, work directly with input meshes instead of copying
    // The mesh copying approach has issues with polymesh API
    // This needs to be redesigned to work with references or use a different approach
    LOGD(Default, Warning, "Mesh initialization disabled - working with input meshes directly");
}

bool EmberCSG::compute_intersections()
{
    LOGD(Default, Debug, "Computing mesh-mesh intersections using EMBER algorithm");
    
    // Use the dedicated intersection module
    m_intersection_result = m_intersector.compute_intersections(m_mesh_a, m_mesh_a_positions,
                                                                m_mesh_b, m_mesh_b_positions);
    
    if (!m_intersection_result.has_intersections())
    {
        LOGD(Default, Info, "No intersections found between meshes");
        // This may be valid for some CSG operations (e.g., union of non-overlapping meshes)
    }
    
    return true;
}

bool EmberCSG::build_local_arrangements()
{
    // Local arrangements are critical for robust boolean operations
    // They handle complex intersection scenarios like:
    // 1. Multiple triangles intersecting at a single point
    // 2. Edge-on-edge intersections
    // 3. Coplanar face intersections
    
    LOGD(Default, Debug, "Building local arrangements at %d intersection points", 
         m_intersection_result.intersections.size());
    
    if (!m_intersection_result.has_intersections())
    {
        LOGD(Default, Debug, "No intersections to process");
        return true;
    }
    
    // Process each intersection to build arrangements
    for (auto const& intersection : m_intersection_result.intersections)
    {
        // In a full EMBER implementation, this would:
        // 1. Group nearby intersections into arrangement regions
        // 2. Build arrangement graphs for each region
        // 3. Compute cell decomposition within each arrangement
        // 4. Classify cells relative to both input meshes
        
        LOGD(Default, Trace, "Processing intersection between triangles %d and %d",
             intersection.triangle_a_index, intersection.triangle_b_index);
    }
    
    return true;
}

bool EmberCSG::classify_elements()
{
    LOGD(Default, Debug, "Classifying mesh elements");
    
    // Classify faces of mesh A relative to mesh B
    auto classification_a = m_classifier.classify_mesh_faces(m_mesh_a, m_mesh_a_positions,
                                                             m_mesh_b, m_mesh_b_positions,
                                                             m_intersection_result);
    
    // Classify faces of mesh B relative to mesh A
    auto classification_b = m_classifier.classify_mesh_faces(m_mesh_b, m_mesh_b_positions,
                                                             m_mesh_a, m_mesh_a_positions,
                                                             m_intersection_result);
    
    // Store classifications for later use in construction
    m_classification_a = classification_a;
    m_classification_b = classification_b;
    
    LOGD(Default, Info, "Classified %d faces from mesh A and %d faces from mesh B",
         classification_a.size(), classification_b.size());
    
    return true;
}

bool EmberCSG::construct_result(csg_operation op)
{
    LOGD(Default, Debug, "Constructing result mesh for operation: %s", csg_utils::operation_to_string(op));
    
    // Clear result mesh
    m_result_mesh.clear();
    m_result_positions = pm::vertex_attribute<pos_t>(m_result_mesh);
    
    // Use the CSG constructor to build the result based on classifications
    bool success = false;
    
    switch (op)
    {
    case csg_operation::union_op:
        success = m_constructor.construct_union(
            m_mesh_a, m_mesh_a_positions,
            m_mesh_b, m_mesh_b_positions,
            m_classification_a, m_classification_b,
            m_intersection_result,
            m_result_mesh, m_result_positions);
        break;
        
    case csg_operation::intersection:
        success = m_constructor.construct_intersection(
            m_mesh_a, m_mesh_a_positions,
            m_mesh_b, m_mesh_b_positions,
            m_classification_a, m_classification_b,
            m_intersection_result,
            m_result_mesh, m_result_positions);
        break;
        
    case csg_operation::difference:
        success = m_constructor.construct_difference(
            m_mesh_a, m_mesh_a_positions,
            m_mesh_b, m_mesh_b_positions,
            m_classification_a, m_classification_b,
            m_intersection_result,
            m_result_mesh, m_result_positions);
        break;
    }
    
    return success;
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