#pragma once

#include <polymesh/Mesh.hh>
#include <integer-plane-geometry/geometry.hh>
#include <integer-plane-geometry/plane.hh>
#include <integer-plane-geometry/intersect.hh>
#include <clean-core/vector.hh>

namespace mk::ember
{

/// Result of triangle-triangle intersection test
struct TriangleIntersection
{
    bool intersects = false;
    
    /// Intersection points (up to 6 for complex cases)
    cc::vector<ipg::geometry<26, 55>::pos_t> points;
    
    /// Intersection type (edge-edge, vertex-face, etc.)
    enum class Type { None, VertexFace, EdgeEdge, EdgeFace, Coplanar } type = Type::None;
    
    /// Triangle indices involved in intersection
    int triangle_a_index = -1;
    int triangle_b_index = -1;
};

/// Result of mesh-mesh intersection computation
struct MeshIntersection
{
    cc::vector<TriangleIntersection> intersections;
    
    /// Intersection curves (sequences of connected intersection points)
    cc::vector<cc::vector<ipg::geometry<26, 55>::pos_t>> curves;
    
    bool has_intersections() const { return !intersections.empty(); }
};

/// EMBER mesh intersection computation
class MeshIntersector
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;
    using plane_t = typename geometry_t::plane_t;

public:
    MeshIntersector() = default;
    
    /// Compute intersections between two meshes
    MeshIntersection compute_intersections(pm::Mesh const& mesh_a,
                                           pm::vertex_attribute<pos_t> const& positions_a,
                                           pm::Mesh const& mesh_b, 
                                           pm::vertex_attribute<pos_t> const& positions_b);

private:
    /// Test intersection between two triangles
    TriangleIntersection intersect_triangles(pm::face_handle face_a,
                                             pm::vertex_attribute<pos_t> const& positions_a,
                                             pm::face_handle face_b,
                                             pm::vertex_attribute<pos_t> const& positions_b);
    
    /// Test if point is inside triangle
    bool point_in_triangle(pos_t const& point,
                          pos_t const& v0, pos_t const& v1, pos_t const& v2);
    
    /// Compute intersection between two line segments
    bool intersect_segments(pos_t const& a0, pos_t const& a1,
                           pos_t const& b0, pos_t const& b1,
                           pos_t& intersection_point);
    
    /// Test if two triangles are coplanar
    bool triangles_coplanar(pm::face_handle face_a,
                           pm::vertex_attribute<pos_t> const& positions_a,
                           pm::face_handle face_b,
                           pm::vertex_attribute<pos_t> const& positions_b);
    
    /// Handle coplanar triangle intersection
    TriangleIntersection intersect_coplanar_triangles(pm::face_handle face_a,
                                                      pm::vertex_attribute<pos_t> const& positions_a,
                                                      pm::face_handle face_b,
                                                      pm::vertex_attribute<pos_t> const& positions_b);
    
    /// Classify vertices of one triangle relative to the plane of another
    cc::vector<int> classify_triangle_vertices(pm::face_handle face,
                                               pm::vertex_attribute<pos_t> const& positions,
                                               plane_t const& plane);
    
    /// Build intersection curves from individual intersection points
    void build_intersection_curves(MeshIntersection& result);
    
    /// Spatial acceleration structure for efficient intersection queries
    struct BVH
    {
        // Simple axis-aligned bounding box tree for triangles
        struct Node
        {
            tg::iaabb3 bbox;
            cc::vector<pm::face_handle> triangles;
            int left_child = -1;
            int right_child = -1;
            bool is_leaf() const { return left_child == -1 && right_child == -1; }
        };
        
        cc::vector<Node> nodes;
        
        void build(pm::Mesh const& mesh, pm::vertex_attribute<pos_t> const& positions);
        void query_intersections(tg::iaabb3 const& query_bbox, cc::vector<pm::face_handle>& results) const;
    private:
        void build_recursive(int node_index, cc::vector<pm::face_handle>& triangles,
                            pm::vertex_attribute<pos_t> const& positions);
        void query_recursive(int node_index, tg::iaabb3 const& query_bbox, 
                            cc::vector<pm::face_handle>& results) const;
    };
    
    BVH m_bvh_a, m_bvh_b;
};

} // namespace mk::ember