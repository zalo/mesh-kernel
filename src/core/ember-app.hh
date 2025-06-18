#pragma once

#include <string>
#include <polymesh/Mesh.hh>
#include <integer-plane-geometry/geometry.hh>
#include <core/ember-csg.hh>

namespace mk
{

/// CLI application for EMBER CSG operations
class EmberApp
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;

public:
    EmberApp() = default;
    
    /// Run the EMBER CLI application
    void run(int argc, char** args);

private: // members
    pm::Mesh m_mesh_a;
    pm::Mesh m_mesh_b;
    pm::vertex_attribute<pos_t> m_positions_a{m_mesh_a};
    pm::vertex_attribute<pos_t> m_positions_b{m_mesh_b};
    
    EmberCSG m_csg_processor;
    ember_options m_ember_options;
    
    std::string m_input_a_path;
    std::string m_input_b_path;
    std::string m_output_path;
    std::string m_operation = "union";
    std::string m_output_format = "obj";

private: // implementation
    /// Parse command line arguments
    bool parse_arguments(int argc, char** args);
    
    /// Load mesh from file
    bool load_mesh(const std::string& path, pm::Mesh& mesh, pm::vertex_attribute<pos_t>& positions);
    
    /// Save mesh to file
    bool save_mesh(const std::string& path, pm::Mesh const& mesh, pm::vertex_attribute<pos_t> const& positions);
    
    /// Print usage information
    void print_usage(const char* program_name);
    
    /// Convert mesh positions to integer coordinates
    void convert_to_integer_positions(pm::vertex_attribute<tg::dpos3> const& input_positions,
                                      pm::vertex_attribute<pos_t>& output_positions,
                                      double scaling_factor = 1000.0);
    
    /// Convert integer positions back to double coordinates
    void convert_to_double_positions(pm::vertex_attribute<pos_t> const& input_positions,
                                     pm::vertex_attribute<tg::dpos3>& output_positions,
                                     double scaling_factor = 1000.0);
    
    /// Get scaling factor for coordinate conversion
    double get_scaling_factor(pm::vertex_attribute<tg::dpos3> const& positions);
};

} // namespace mk