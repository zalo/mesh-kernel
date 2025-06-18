#include "ember-classify.hh"

#include <rich-log/log.hh>
#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/plane.hh>
#include <typed-geometry/functions/vector/cross.hh>
#include <typed-geometry/functions/vector/dot.hh>
#include <clean-core/set.hh>
#include <clean-core/map.hh>

namespace mk::ember
{

cc::vector<ElementClassificationResult> MeshClassifier::classify_mesh_faces(
    pm::Mesh const& mesh_a,
    pm::vertex_attribute<pos_t> const& positions_a,
    pm::Mesh const& mesh_b,
    pm::vertex_attribute<pos_t> const& positions_b,
    MeshIntersection const& intersections)
{
    cc::vector<ElementClassificationResult> results;
    results.reserve(mesh_a.faces().size());
    
    LOGD(Default, Debug, "Classifying %d faces of mesh A relative to mesh B", mesh_a.faces().size());
    
    for (auto face : mesh_a.faces())
    {
        // Check if face is on boundary first
        if (face_on_boundary(face, intersections))
        {
            ElementClassificationResult result;
            result.classification = ElementClassification::OnBoundary;
            result.is_certain = true;
            results.push_back(result);
        }
        else
        {
            // Classify face based on its centroid
            auto result = classify_face(face, positions_a, mesh_b, positions_b);
            results.push_back(result);
        }
    }
    
    LOGD(Default, Debug, "Classification complete");
    return results;
}

ElementClassificationResult MeshClassifier::classify_face(
    pm::face_handle face,
    pm::vertex_attribute<pos_t> const& face_positions,
    pm::Mesh const& other_mesh,
    pm::vertex_attribute<pos_t> const& other_positions)
{
    // Get face centroid for classification
    auto centroid = get_face_centroid(face, face_positions);
    
    // Use point-in-mesh test
    return point_in_mesh(centroid, other_mesh, other_positions);
}

ElementClassificationResult MeshClassifier::point_in_mesh(
    pos_t const& point,
    pm::Mesh const& mesh,
    pm::vertex_attribute<pos_t> const& positions)
{
    ElementClassificationResult result;
    
    // Use ray casting algorithm with multiple rays for robustness
    cc::vector<pos_t> ray_directions = {
        pos_t(1, 0, 0),   // +X direction
        pos_t(0, 1, 0),   // +Y direction  
        pos_t(0, 0, 1),   // +Z direction
        pos_t(1, 1, 1),   // Diagonal direction
    };
    
    cc::vector<int> intersection_counts;
    intersection_counts.reserve(ray_directions.size());
    
    for (auto const& direction : ray_directions)
    {
        int count = count_ray_intersections(point, direction, mesh, positions);
        intersection_counts.push_back(count);
    }
    
    // Determine classification based on intersection counts
    // If most rays have odd intersection count, point is inside
    int odd_count = 0;
    int even_count = 0;
    
    for (int count : intersection_counts)
    {
        if (count % 2 == 1)
            odd_count++;
        else
            even_count++;
    }
    
    if (odd_count > even_count)
    {
        result.classification = ElementClassification::Inside;
        result.is_certain = (odd_count == intersection_counts.size()); // All rays agree
    }
    else
    {
        result.classification = ElementClassification::Outside;
        result.is_certain = (even_count == intersection_counts.size()); // All rays agree
    }
    
    return result;
}

int MeshClassifier::count_ray_intersections(
    pos_t const& point,
    pos_t const& direction,
    pm::Mesh const& mesh,
    pm::vertex_attribute<pos_t> const& positions)
{
    int intersection_count = 0;
    
    for (auto face : mesh.faces())
    {
        auto vertices = face.vertices().to_vector();
        if (vertices.size() != 3) continue; // Only handle triangular faces
        
        pos_t v0 = positions[vertices[0]];
        pos_t v1 = positions[vertices[1]];
        pos_t v2 = positions[vertices[2]];
        
        pos_t intersection_point;
        if (ray_triangle_intersect(point, direction, v0, v1, v2, intersection_point))
        {
            intersection_count++;
        }
    }
    
    return intersection_count;
}

bool MeshClassifier::ray_triangle_intersect(
    pos_t const& ray_origin,
    pos_t const& ray_direction,
    pos_t const& v0, pos_t const& v1, pos_t const& v2,
    pos_t& intersection_point)
{
    // Möller-Trumbore ray-triangle intersection algorithm
    // Adapted for integer arithmetic
    
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;
    
    // Compute cross product of ray direction and edge2
    auto h = cross(ray_direction, edge2);
    auto a = dot(edge1, h);
    
    // Check if ray is parallel to triangle
    if (a == 0) return false;
    
    auto f = 1.0 / double(a); // Use double for division
    auto s = ray_origin - v0;
    auto u = f * double(dot(s, h));
    
    if (u < 0.0 || u > 1.0) return false;
    
    auto q = cross(s, edge1);
    auto v = f * double(dot(ray_direction, q));
    
    if (v < 0.0 || u + v > 1.0) return false;
    
    // Check if intersection is in front of ray origin
    auto t = f * double(dot(edge2, q));
    
    if (t > 1e-6) // Ray intersects triangle
    {
        // Compute intersection point
        intersection_point = ray_origin + pos_t(
            static_cast<int64_t>(t * ray_direction.x),
            static_cast<int64_t>(t * ray_direction.y),
            static_cast<int64_t>(t * ray_direction.z)
        );
        return true;
    }
    
    return false;
}

pos_t MeshClassifier::get_face_centroid(
    pm::face_handle face,
    pm::vertex_attribute<pos_t> const& positions)
{
    auto vertices = face.vertices().to_vector();
    if (vertices.empty()) return pos_t(0, 0, 0);
    
    pos_t centroid(0, 0, 0);
    for (auto v : vertices)
    {
        centroid += positions[v];
    }
    
    // Integer division for centroid
    int64_t count = vertices.size();
    return pos_t(centroid.x / count, centroid.y / count, centroid.z / count);
}

bool MeshClassifier::face_on_boundary(
    pm::face_handle face,
    MeshIntersection const& intersections)
{
    int face_index = face.idx.value;
    
    for (auto const& intersection : intersections.intersections)
    {
        if (intersection.triangle_a_index == face_index || 
            intersection.triangle_b_index == face_index)
        {
            return true;
        }
    }
    
    return false;
}

// CSGConstructor implementation

bool CSGConstructor::construct_union(
    pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
    pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
    cc::vector<ElementClassificationResult> const& classification_a,
    cc::vector<ElementClassificationResult> const& classification_b,
    MeshIntersection const& intersections,
    pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions)
{
    LOGD(Default, Debug, "Constructing union result");
    
    result_mesh.clear();
    result_positions = pm::vertex_attribute<pos_t>(result_mesh);
    
    // For union: include faces from A that are outside or on boundary of B
    //           include faces from B that are outside or on boundary of A
    auto include_a = select_faces_for_union(classification_a);
    auto include_b = select_faces_for_union(classification_b);
    
    // Copy selected faces from mesh A
    copy_faces_to_result(mesh_a, positions_a, include_a, result_mesh, result_positions);
    
    // Copy selected faces from mesh B
    copy_faces_to_result(mesh_b, positions_b, include_b, result_mesh, result_positions);
    
    // Add intersection geometry if needed
    add_intersection_geometry(intersections, result_mesh, result_positions);
    
    LOGD(Default, Info, "Union result: %d vertices, %d faces", 
         result_mesh.vertices().size(), result_mesh.faces().size());
    
    return true;
}

bool CSGConstructor::construct_intersection(
    pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
    pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
    cc::vector<ElementClassificationResult> const& classification_a,
    cc::vector<ElementClassificationResult> const& classification_b,
    MeshIntersection const& intersections,
    pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions)
{
    LOGD(Default, Debug, "Constructing intersection result");
    
    result_mesh.clear();
    result_positions = pm::vertex_attribute<pos_t>(result_mesh);
    
    // For intersection: include faces from A that are inside or on boundary of B
    //                  include faces from B that are inside or on boundary of A
    auto include_a = select_faces_for_intersection(classification_a);
    auto include_b = select_faces_for_intersection(classification_b);
    
    // Copy selected faces
    copy_faces_to_result(mesh_a, positions_a, include_a, result_mesh, result_positions);
    copy_faces_to_result(mesh_b, positions_b, include_b, result_mesh, result_positions);
    
    // Add intersection geometry
    add_intersection_geometry(intersections, result_mesh, result_positions);
    
    LOGD(Default, Info, "Intersection result: %d vertices, %d faces",
         result_mesh.vertices().size(), result_mesh.faces().size());
    
    return true;
}

bool CSGConstructor::construct_difference(
    pm::Mesh const& mesh_a, pm::vertex_attribute<pos_t> const& positions_a,
    pm::Mesh const& mesh_b, pm::vertex_attribute<pos_t> const& positions_b,
    cc::vector<ElementClassificationResult> const& classification_a,
    cc::vector<ElementClassificationResult> const& classification_b,
    MeshIntersection const& intersections,
    pm::Mesh& result_mesh, pm::vertex_attribute<pos_t>& result_positions)
{
    LOGD(Default, Debug, "Constructing difference result");
    
    result_mesh.clear();
    result_positions = pm::vertex_attribute<pos_t>(result_mesh);
    
    // For difference (A - B): include faces from A that are outside or on boundary of B
    //                        include faces from B that are inside A (with flipped normals)
    auto include_a = select_faces_for_difference(classification_a, false);
    auto include_b = select_faces_for_difference(classification_b, true);
    
    // Copy selected faces from A
    copy_faces_to_result(mesh_a, positions_a, include_a, result_mesh, result_positions);
    
    // Copy selected faces from B (these represent the "cavity" left by subtraction)
    copy_faces_to_result(mesh_b, positions_b, include_b, result_mesh, result_positions);
    
    // Add intersection geometry
    add_intersection_geometry(intersections, result_mesh, result_positions);
    
    LOGD(Default, Info, "Difference result: %d vertices, %d faces",
         result_mesh.vertices().size(), result_mesh.faces().size());
    
    return true;
}

void CSGConstructor::copy_faces_to_result(
    pm::Mesh const& source_mesh,
    pm::vertex_attribute<pos_t> const& source_positions,
    cc::vector<bool> const& include_face,
    pm::Mesh& result_mesh,
    pm::vertex_attribute<pos_t>& result_positions)
{
    // Map from source vertex handles to result vertex handles
    cc::set<pm::vertex_handle> used_vertices;
    
    // First pass: identify used vertices
    int face_index = 0;
    for (auto face : source_mesh.faces())
    {
        if (face_index < include_face.size() && include_face[face_index])
        {
            for (auto v : face.vertices())
            {
                used_vertices.insert(v);
            }
        }
        face_index++;
    }
    
    // Create vertex mapping
    cc::map<pm::vertex_handle, pm::vertex_handle> vertex_map;
    for (auto v : used_vertices)
    {
        auto new_v = result_mesh.vertices().add();
        vertex_map[v] = new_v;
        result_positions[new_v] = source_positions[v];
    }
    
    // Second pass: copy faces
    face_index = 0;
    for (auto face : source_mesh.faces())
    {
        if (face_index < include_face.size() && include_face[face_index])
        {
            auto vertices = face.vertices().to_vector();
            cc::vector<pm::vertex_handle> new_vertices;
            
            for (auto v : vertices)
            {
                new_vertices.push_back(vertex_map[v]);
            }
            
            result_mesh.faces().add(new_vertices);
        }
        face_index++;
    }
}

void CSGConstructor::add_intersection_geometry(
    MeshIntersection const& intersections,
    pm::Mesh& result_mesh,
    pm::vertex_attribute<pos_t>& result_positions)
{
    // Add geometry from intersection curves
    // This is complex and would require careful stitching of intersection geometry
    LOGD(Default, Debug, "Adding intersection geometry (placeholder)");
}

cc::vector<bool> CSGConstructor::select_faces_for_union(
    cc::vector<ElementClassificationResult> const& classification)
{
    cc::vector<bool> include(classification.size(), false);
    
    for (size_t i = 0; i < classification.size(); ++i)
    {
        auto const& c = classification[i];
        // Include faces that are outside or on boundary
        include[i] = (c.classification == ElementClassification::Outside ||
                     c.classification == ElementClassification::OnBoundary);
    }
    
    return include;
}

cc::vector<bool> CSGConstructor::select_faces_for_intersection(
    cc::vector<ElementClassificationResult> const& classification)
{
    cc::vector<bool> include(classification.size(), false);
    
    for (size_t i = 0; i < classification.size(); ++i)
    {
        auto const& c = classification[i];
        // Include faces that are inside or on boundary
        include[i] = (c.classification == ElementClassification::Inside ||
                     c.classification == ElementClassification::OnBoundary);
    }
    
    return include;
}

cc::vector<bool> CSGConstructor::select_faces_for_difference(
    cc::vector<ElementClassificationResult> const& classification,
    bool is_subtracted_mesh)
{
    cc::vector<bool> include(classification.size(), false);
    
    for (size_t i = 0; i < classification.size(); ++i)
    {
        auto const& c = classification[i];
        
        if (is_subtracted_mesh)
        {
            // For subtracted mesh: include faces that are inside (they form cavities)
            include[i] = (c.classification == ElementClassification::Inside ||
                         c.classification == ElementClassification::OnBoundary);
        }
        else
        {
            // For main mesh: include faces that are outside or on boundary
            include[i] = (c.classification == ElementClassification::Outside ||
                         c.classification == ElementClassification::OnBoundary);
        }
    }
    
    return include;
}

} // namespace mk::ember