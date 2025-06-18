#include "ember-intersect.hh"

#include <algorithm>
#include <rich-log/log.hh>
#include <integer-plane-geometry/classify.hh>
#include <integer-plane-geometry/are_parallel.hh>
#include <typed-geometry/functions/basic/scalar_math.hh>
#include <typed-geometry/functions/basic/minmax.hh>
#include <typed-geometry/functions/basic/intersections.hh>

namespace mk::ember
{

MeshIntersection MeshIntersector::compute_intersections(pm::Mesh const& mesh_a,
                                                        pm::vertex_attribute<pos_t> const& positions_a,
                                                        pm::Mesh const& mesh_b,
                                                        pm::vertex_attribute<pos_t> const& positions_b)
{
    LOGD(Default, Debug, "Computing mesh-mesh intersections");
    
    MeshIntersection result;
    
    // Build spatial acceleration structures
    m_bvh_a.build(mesh_a, positions_a);
    m_bvh_b.build(mesh_b, positions_b);
    
    // Test each triangle in mesh A against potentially intersecting triangles in mesh B
    for (auto face_a : mesh_a.faces())
    {
        // Compute bounding box for triangle A
        auto vertices_a = face_a.vertices().to_vector();
        if (vertices_a.size() != 3) continue; // Skip non-triangular faces
        
        tg::iaabb3 bbox_a;
        bbox_a.min = positions_a[vertices_a[0]];
        bbox_a.max = positions_a[vertices_a[0]];
        
        for (int i = 1; i < 3; ++i)
        {
            auto pos = positions_a[vertices_a[i]];
            bbox_a.min = tg::min(bbox_a.min, pos);
            bbox_a.max = tg::max(bbox_a.max, pos);
        }
        
        // Query potentially intersecting triangles from mesh B
        cc::vector<pm::face_handle> candidates;
        m_bvh_b.query_intersections(bbox_a, candidates);
        
        // Test intersection with each candidate
        for (auto face_b : candidates)
        {
            auto intersection = intersect_triangles(face_a, positions_a, face_b, positions_b);
            if (intersection.intersects)
            {
                result.intersections.push_back(intersection);
            }
        }
    }
    
    // Build intersection curves from individual intersection points
    build_intersection_curves(result);
    
    LOGD(Default, Info, "Found %d triangle-triangle intersections", result.intersections.size());
    return result;
}

TriangleIntersection MeshIntersector::intersect_triangles(pm::face_handle face_a,
                                                          pm::vertex_attribute<pos_t> const& positions_a,
                                                          pm::face_handle face_b,
                                                          pm::vertex_attribute<pos_t> const& positions_b)
{
    TriangleIntersection result;
    result.triangle_a_index = face_a.idx.value;
    result.triangle_b_index = face_b.idx.value;
    
    // Get triangle vertices
    auto vertices_a = face_a.vertices().to_vector();
    auto vertices_b = face_b.vertices().to_vector();
    
    if (vertices_a.size() != 3 || vertices_b.size() != 3)
        return result; // Only handle triangular faces
    
    pos_t v0_a = positions_a[vertices_a[0]];
    pos_t v1_a = positions_a[vertices_a[1]];
    pos_t v2_a = positions_a[vertices_a[2]];
    
    pos_t v0_b = positions_b[vertices_b[0]];
    pos_t v1_b = positions_b[vertices_b[1]];
    pos_t v2_b = positions_b[vertices_b[2]];
    
    // Compute plane equations
    plane_t plane_a = plane_t::from_points(v0_a, v1_a, v2_a);
    plane_t plane_b = plane_t::from_points(v0_b, v1_b, v2_b);
    
    // Check if triangles are coplanar
    if (triangles_coplanar(face_a, positions_a, face_b, positions_b))
    {
        return intersect_coplanar_triangles(face_a, positions_a, face_b, positions_b);
    }
    
    // Classify vertices of triangle A relative to plane B
    auto class_a = classify_triangle_vertices(face_a, positions_a, plane_b);
    
    // Classify vertices of triangle B relative to plane A  
    auto class_b = classify_triangle_vertices(face_b, positions_b, plane_a);
    
    // Check if triangles are separated by their planes
    bool all_a_positive = true, all_a_negative = true;
    bool all_b_positive = true, all_b_negative = true;
    
    for (int c : class_a)
    {
        if (c <= 0) all_a_positive = false;
        if (c >= 0) all_a_negative = false;
    }
    
    for (int c : class_b)
    {
        if (c <= 0) all_b_positive = false;
        if (c >= 0) all_b_negative = false;
    }
    
    if (all_a_positive || all_a_negative || all_b_positive || all_b_negative)
        return result; // No intersection
    
    // If we get here, the triangles intersect
    result.intersects = true;
    result.type = TriangleIntersection::Type::EdgeEdge; // Simplified for now
    
    // Compute intersection line between the two planes
    auto intersection_line = ipg::intersect(plane_a, plane_b);
    
    // Find intersection points on triangle edges
    // This is a simplified implementation - full EMBER would handle all edge cases
    
    return result;
}

bool MeshIntersector::point_in_triangle(pos_t const& point,
                                        pos_t const& v0, pos_t const& v1, pos_t const& v2)
{
    // Use barycentric coordinates to test if point is inside triangle
    // This is a simplified implementation using exact arithmetic
    
    // Compute vectors
    auto v0v1 = v1 - v0;
    auto v0v2 = v2 - v0; 
    auto v0p = point - v0;
    
    // For exact arithmetic with integer coordinates, we'll use a simpler approach
    // Check if point is on the same side of each edge as the opposite vertex
    
    // Helper function to compute cross product sign
    auto cross_sign = [](auto const& a, auto const& b, auto const& c, auto const& d) -> int
    {
        // Cross product of vectors (b-a) and (d-c)
        auto dx1 = b.x - a.x;
        auto dy1 = b.y - a.y;
        auto dx2 = d.x - c.x; 
        auto dy2 = d.y - c.y;
        
        auto cross = dx1 * dy2 - dy1 * dx2;
        return tg::sign(cross);
    };
    
    // Project to 2D by dropping the coordinate with largest normal component
    // For simplicity, just use XY projection
    auto p_2d = tg::ipos2(point.x, point.y);
    auto v0_2d = tg::ipos2(v0.x, v0.y);
    auto v1_2d = tg::ipos2(v1.x, v1.y);
    auto v2_2d = tg::ipos2(v2.x, v2.y);
    
    // Check if point is on same side of each edge
    int sign1 = cross_sign(v0_2d, v1_2d, v0_2d, p_2d);
    int sign2 = cross_sign(v1_2d, v2_2d, v1_2d, p_2d);
    int sign3 = cross_sign(v2_2d, v0_2d, v2_2d, p_2d);
    
    return (sign1 >= 0 && sign2 >= 0 && sign3 >= 0) || (sign1 <= 0 && sign2 <= 0 && sign3 <= 0);
}

bool MeshIntersector::intersect_segments(pos_t const& a0, pos_t const& a1,
                                         pos_t const& b0, pos_t const& b1,
                                         pos_t& intersection_point)
{
    // Compute intersection between two 3D line segments
    // This is a simplified implementation - full EMBER would handle all cases
    
    auto dir_a = a1 - a0;
    auto dir_b = b1 - b0;
    auto start_diff = a0 - b0;
    
    // Check if segments are parallel
    if (ipg::are_parallel(dir_a, dir_b))
        return false;
    
    // For now, return false - full implementation would compute intersection
    return false;
}

bool MeshIntersector::triangles_coplanar(pm::face_handle face_a,
                                         pm::vertex_attribute<pos_t> const& positions_a,
                                         pm::face_handle face_b,
                                         pm::vertex_attribute<pos_t> const& positions_b)
{
    // Get triangle vertices
    auto vertices_a = face_a.vertices().to_vector();
    auto vertices_b = face_b.vertices().to_vector();
    
    if (vertices_a.size() != 3 || vertices_b.size() != 3)
        return false;
    
    // Compute plane of triangle A
    pos_t v0_a = positions_a[vertices_a[0]];
    pos_t v1_a = positions_a[vertices_a[1]];
    pos_t v2_a = positions_a[vertices_a[2]];
    
    plane_t plane_a = plane_t::from_points(v0_a, v1_a, v2_a);
    
    // Test if all vertices of triangle B lie on plane A
    for (auto v : vertices_b)
    {
        auto classification = ipg::classify(positions_b[v], plane_a);
        if (classification != 0)
            return false;
    }
    
    return true;
}

TriangleIntersection MeshIntersector::intersect_coplanar_triangles(pm::face_handle face_a,
                                                                   pm::vertex_attribute<pos_t> const& positions_a,
                                                                   pm::face_handle face_b,
                                                                   pm::vertex_attribute<pos_t> const& positions_b)
{
    TriangleIntersection result;
    result.triangle_a_index = face_a.idx.value;
    result.triangle_b_index = face_b.idx.value;
    result.type = TriangleIntersection::Type::Coplanar;
    
    // For coplanar triangles, we need to compute their 2D intersection
    // This is a complex operation that would require projecting to 2D
    // For now, mark as intersecting if any vertex of one triangle is inside the other
    
    auto vertices_a = face_a.vertices().to_vector();
    auto vertices_b = face_b.vertices().to_vector();
    
    if (vertices_a.size() != 3 || vertices_b.size() != 3)
        return result;
    
    pos_t v0_a = positions_a[vertices_a[0]];
    pos_t v1_a = positions_a[vertices_a[1]];
    pos_t v2_a = positions_a[vertices_a[2]];
    
    pos_t v0_b = positions_b[vertices_b[0]];
    pos_t v1_b = positions_b[vertices_b[1]];
    pos_t v2_b = positions_b[vertices_b[2]];
    
    // Check if any vertex of B is inside triangle A
    if (point_in_triangle(v0_b, v0_a, v1_a, v2_a) ||
        point_in_triangle(v1_b, v0_a, v1_a, v2_a) ||
        point_in_triangle(v2_b, v0_a, v1_a, v2_a))
    {
        result.intersects = true;
    }
    
    // Check if any vertex of A is inside triangle B
    if (point_in_triangle(v0_a, v0_b, v1_b, v2_b) ||
        point_in_triangle(v1_a, v0_b, v1_b, v2_b) ||
        point_in_triangle(v2_a, v0_b, v1_b, v2_b))
    {
        result.intersects = true;
    }
    
    return result;
}

cc::vector<int> MeshIntersector::classify_triangle_vertices(pm::face_handle face,
                                                           pm::vertex_attribute<pos_t> const& positions,
                                                           plane_t const& plane)
{
    cc::vector<int> classifications;
    
    for (auto v : face.vertices())
    {
        auto classification = ipg::classify(positions[v], plane);
        classifications.push_back(classification);
    }
    
    return classifications;
}

void MeshIntersector::build_intersection_curves(MeshIntersection& result)
{
    // Build connected curves from intersection points
    // This is a complex operation that would require careful connectivity analysis
    // For now, this is a placeholder
    
    LOGD(Default, Debug, "Building intersection curves from %d intersections", result.intersections.size());
}

// BVH Implementation
void MeshIntersector::BVH::build(pm::Mesh const& mesh, pm::vertex_attribute<pos_t> const& positions)
{
    nodes.clear();
    
    // Collect all triangular faces
    cc::vector<pm::face_handle> triangles;
    for (auto f : mesh.faces())
    {
        if (f.vertices().size() == 3)
            triangles.push_back(f);
    }
    
    if (triangles.empty())
        return;
    
    // Create root node
    nodes.resize(1);
    nodes[0].triangles = triangles;
    
    // Compute root bounding box
    bool first = true;
    for (auto f : triangles)
    {
        for (auto v : f.vertices())
        {
            auto pos = positions[v];
            if (first)
            {
                nodes[0].bbox.min = pos;
                nodes[0].bbox.max = pos;
                first = false;
            }
            else
            {
                nodes[0].bbox.min = tg::min(nodes[0].bbox.min, pos);
                nodes[0].bbox.max = tg::max(nodes[0].bbox.max, pos);
            }
        }
    }
    
    // Build tree recursively
    build_recursive(0, triangles, positions);
}

void MeshIntersector::BVH::build_recursive(int node_index, cc::vector<pm::face_handle>& triangles,
                                          pm::vertex_attribute<pos_t> const& positions)
{
    // Stop if few triangles or max depth reached
    if (triangles.size() <= 4 || nodes.size() > 1000)
        return;
    
    // Find longest axis and sort triangles
    auto bbox = nodes[node_index].bbox;
    auto size = bbox.max - bbox.min;
    int longest_axis = 0;
    if (size.y > size.x) longest_axis = 1;
    if (size.z > size[longest_axis]) longest_axis = 2;
    
    // Sort triangles along longest axis by centroid
    std::sort(triangles.begin(), triangles.end(), 
        [&](pm::face_handle a, pm::face_handle b)
        {
            // Compute centroids
            auto vertices_a = a.vertices().to_vector();
            auto vertices_b = b.vertices().to_vector();
            
            if (vertices_a.size() != 3 || vertices_b.size() != 3)
                return false;
            
            auto centroid_a = (positions[vertices_a[0]] + positions[vertices_a[1]] + positions[vertices_a[2]]);
            auto centroid_b = (positions[vertices_b[0]] + positions[vertices_b[1]] + positions[vertices_b[2]]);
            
            return centroid_a[longest_axis] < centroid_b[longest_axis];
        });
    
    // Split triangles in half
    int mid = triangles.size() / 2;
    cc::vector<pm::face_handle> left_triangles(triangles.begin(), triangles.begin() + mid);
    cc::vector<pm::face_handle> right_triangles(triangles.begin() + mid, triangles.end());
    
    // Create child nodes
    int left_index = nodes.size();
    int right_index = nodes.size() + 1;
    
    nodes[node_index].left_child = left_index;
    nodes[node_index].right_child = right_index;
    nodes[node_index].triangles.clear(); // No longer a leaf
    
    nodes.resize(nodes.size() + 2);
    nodes[left_index].triangles = left_triangles;
    nodes[right_index].triangles = right_triangles;
    
    // Compute child bounding boxes and recurse
    // Left child bbox
    bool first = true;
    for (auto f : left_triangles)
    {
        for (auto v : f.vertices())
        {
            auto pos = positions[v];
            if (first)
            {
                nodes[left_index].bbox.min = pos;
                nodes[left_index].bbox.max = pos;
                first = false;
            }
            else
            {
                nodes[left_index].bbox.min = tg::min(nodes[left_index].bbox.min, pos);
                nodes[left_index].bbox.max = tg::max(nodes[left_index].bbox.max, pos);
            }
        }
    }
    
    // Right child bbox
    first = true;
    for (auto f : right_triangles)
    {
        for (auto v : f.vertices())
        {
            auto pos = positions[v];
            if (first)
            {
                nodes[right_index].bbox.min = pos;
                nodes[right_index].bbox.max = pos;
                first = false;
            }
            else
            {
                nodes[right_index].bbox.min = tg::min(nodes[right_index].bbox.min, pos);
                nodes[right_index].bbox.max = tg::max(nodes[right_index].bbox.max, pos);
            }
        }
    }
    
    // Recurse on children
    build_recursive(left_index, left_triangles, positions);
    build_recursive(right_index, right_triangles, positions);
}

void MeshIntersector::BVH::query_intersections(tg::iaabb3 const& query_bbox, cc::vector<pm::face_handle>& results) const
{
    if (nodes.empty())
        return;
    
    query_recursive(0, query_bbox, results);
}

void MeshIntersector::BVH::query_recursive(int node_index, tg::iaabb3 const& query_bbox,
                                           cc::vector<pm::face_handle>& results) const
{
    if (node_index >= nodes.size())
        return;
    
    auto const& node = nodes[node_index];
    
    // Check if query bbox intersects node bbox
    if (!intersects(query_bbox, node.bbox))
        return;
    
    if (node.is_leaf())
    {
        // Add all triangles in this leaf
        for (auto f : node.triangles)
            results.push_back(f);
    }
    else
    {
        // Recurse on children
        query_recursive(node.left_child, query_bbox, results);
        query_recursive(node.right_child, query_bbox, results);
    }
}

} // namespace mk::ember