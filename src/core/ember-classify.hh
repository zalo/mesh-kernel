#pragma once

#include <polymesh/Mesh.hh>
#include <integer-plane-geometry/geometry.hh>
#include <core/ember-intersect.hh>
#include <clean-core/vector.hh>
#include <clean-core/map.hh>

namespace mk::ember
{

/// Classification of mesh elements relative to another mesh
enum class ElementClassification
{
    Inside,    // Element is inside the other mesh
    Outside,   // Element is outside the other mesh
    OnBoundary // Element is on the boundary (intersecting)
};

/// Classification result for a mesh element
struct ElementClassificationResult
{
    ElementClassification classification = ElementClassification::Outside;
    bool is_certain = false; // True if classification is certain, false if uncertain
};

/// Classification of mesh faces for CSG operations
class MeshClassifier
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;
    using plane_t = typename geometry_t::plane_t;

public:
    MeshClassifier() = default;
    
    /// Classify all faces of mesh A relative to mesh B
    cc::vector<ElementClassificationResult> classify_mesh_faces(
        pm::Mesh const& mesh_a,
        pm::vertex_attribute<pos_t> const& positions_a,
        pm::Mesh const& mesh_b,
        pm::vertex_attribute<pos_t> const& positions_b,
        MeshIntersection const& intersections);
    
    /// Classify a single face relative to a mesh
    ElementClassificationResult classify_face(
        pm::face_handle face,
        pm::vertex_attribute<pos_t> const& face_positions,
        pm::Mesh const& other_mesh,
        pm::vertex_attribute<pos_t> const& other_positions);

private:
    /// Test if a point is inside a mesh using ray casting
    ElementClassificationResult point_in_mesh(
        pos_t const& point,
        pm::Mesh const& mesh,
        pm::vertex_attribute<pos_t> const& positions);
    
    /// Cast a ray from point and count intersections with mesh
    int count_ray_intersections(
        pos_t const& point,
        pos_t const& direction,
        pm::Mesh const& mesh,
        pm::vertex_attribute<pos_t> const& positions);
    
    /// Test ray-triangle intersection
    bool ray_triangle_intersect(
        pos_t const& ray_origin,
        pos_t const& ray_direction,
        pos_t const& v0, pos_t const& v1, pos_t const& v2,
        pos_t& intersection_point);
    
    /// Get face centroid
    pos_t get_face_centroid(
        pm::face_handle face,
        pm::vertex_attribute<pos_t> const& positions);
    
    /// Check if face is on the boundary (intersects other mesh)
    bool face_on_boundary(
        pm::face_handle face,
        MeshIntersection const& intersections);
};

/// CSG result construction based on element classifications
class CSGConstructor
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;

public:
    CSGConstructor() = default;
    
    /// Construct union of two meshes
    bool construct_union(
        pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
        pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
        cc::vector<ElementClassificationResult> const& classification_a,
        cc::vector<ElementClassificationResult> const& classification_b,
        MeshIntersection const& intersections,
        pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions);
    
    /// Construct intersection of two meshes
    bool construct_intersection(
        pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
        pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
        cc::vector<ElementClassificationResult> const& classification_a,
        cc::vector<ElementClassificationResult> const& classification_b,
        MeshIntersection const& intersections,
        pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions);
    
    /// Construct difference of two meshes (A - B)
    bool construct_difference(
        pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
        pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
        cc::vector<ElementClassificationResult> const& classification_a,
        cc::vector<ElementClassificationResult> const& classification_b,
        MeshIntersection const& intersections,
        pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions);

private:
    /// Copy selected faces from source mesh to result mesh
    void copy_faces_to_result(
        pm::Mesh const& source_mesh,
        pm::vertex_attribute<pos_t> const& source_positions,
        cc::vector<bool> const& include_face,
        pm::Mesh& result_mesh,
        pm::vertex_attribute<pos_t>& result_positions);
    
    /// Add intersection geometry to result mesh
    void add_intersection_geometry(
        MeshIntersection const& intersections,
        pm::Mesh& result_mesh,
        pm::vertex_attribute<pos_t>& result_positions);
    
    /// Determine which faces to include for union operation
    cc::vector<bool> select_faces_for_union(
        cc::vector<ElementClassificationResult> const& classification);
    
    /// Determine which faces to include for intersection operation
    cc::vector<bool> select_faces_for_intersection(
        cc::vector<ElementClassificationResult> const& classification);
    
    /// Determine which faces to include for difference operation
    cc::vector<bool> select_faces_for_difference(
        cc::vector<ElementClassificationResult> const& classification,
        bool is_subtracted_mesh);
};

} // namespace mk::ember