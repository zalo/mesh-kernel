#include "ember-app.hh"

#include <iostream>
#include <filesystem>
#include <cstring>

#include <clean-core/format.hh>
#include <rich-log/log.hh>

#include <polymesh/formats.hh>
#include <polymesh/algorithms/normalize.hh>
#include <polymesh/algorithms/deduplicate.hh>

#include <integer-plane-geometry/point.hh>

namespace mk
{

void EmberApp::run(int argc, char** args)
{
    if (!parse_arguments(argc, args))
    {
        print_usage(args[0]);
        return;
    }
    
    LOGD(Default, Info, "Loading mesh A from: %s", m_input_a_path.c_str());
    if (!load_mesh(m_input_a_path, m_mesh_a, m_positions_a))
    {
        LOGD(Default, Error, "Failed to load mesh A");
        return;
    }
    
    LOGD(Default, Info, "Loading mesh B from: %s", m_input_b_path.c_str());
    if (!load_mesh(m_input_b_path, m_mesh_b, m_positions_b))
    {
        LOGD(Default, Error, "Failed to load mesh B");
        return;
    }
    
    // Perform CSG operation
    csg_operation op = csg_utils::string_to_operation(m_operation.c_str());
    
    LOGD(Default, Info, "Performing CSG operation: %s", m_operation.c_str());
    bool success = m_csg_processor.compute_csg(m_positions_a, m_positions_b, op, m_ember_options);
    
    if (!success)
    {
        LOGD(Default, Error, "CSG operation failed");
        return;
    }
    
    // Save result
    LOGD(Default, Info, "Saving result to: %s", m_output_path.c_str());
    if (!save_mesh(m_output_path, m_csg_processor.result_mesh(), m_csg_processor.result_positions()))
    {
        LOGD(Default, Error, "Failed to save result mesh");
        return;
    }
    
    LOGD(Default, Info, "EMBER CSG operation completed successfully");
}

bool EmberApp::parse_arguments(int argc, char** args)
{
    if (argc < 2)
        return false;
    
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(args[i], "-a") == 0 || strcmp(args[i], "--input-a") == 0)
        {
            if (i + 1 >= argc) return false;
            m_input_a_path = args[++i];
        }
        else if (strcmp(args[i], "-b") == 0 || strcmp(args[i], "--input-b") == 0)
        {
            if (i + 1 >= argc) return false;
            m_input_b_path = args[++i];
        }
        else if (strcmp(args[i], "-o") == 0 || strcmp(args[i], "--output") == 0)
        {
            if (i + 1 >= argc) return false;
            m_output_path = args[++i];
        }
        else if (strcmp(args[i], "--operation") == 0)
        {
            if (i + 1 >= argc) return false;
            m_operation = args[++i];
        }
        else if (strcmp(args[i], "--format") == 0)
        {
            if (i + 1 >= argc) return false;
            m_output_format = args[++i];
        }
        else if (strcmp(args[i], "--no-exact") == 0)
        {
            m_ember_options.use_exact_arithmetic = false;
        }
        else if (strcmp(args[i], "--no-validate") == 0)
        {
            m_ember_options.validate_results = false;
        }
        else if (strcmp(args[i], "-h") == 0 || strcmp(args[i], "--help") == 0)
        {
            return false;
        }
    }
    
    // Check required arguments
    if (m_input_a_path.empty() || m_input_b_path.empty() || m_output_path.empty())
        return false;
    
    // Validate operation
    if (m_operation != "union" && m_operation != "intersection" && m_operation != "difference")
    {
        LOGD(Default, Error, "Invalid operation: %s. Must be union, intersection, or difference", m_operation.c_str());
        return false;
    }
    
    return true;
}

bool EmberApp::load_mesh(const std::string& path, pm::Mesh& mesh, pm::vertex_attribute<pos_t>& positions)
{
    try
    {
        // Load mesh using polymesh
        auto temp_positions = pm::vertex_attribute<tg::dpos3>(mesh);
        
        if (!pm::load(path, mesh, temp_positions))
        {
            LOGD(Default, Error, "Failed to load mesh from %s", path.c_str());
            return false;
        }
        
        // Note: Deduplication disabled due to hash function issues with tg::dpos3
        // pm::deduplicate(mesh, temp_positions);
        
        // Convert to integer positions for exact arithmetic
        convert_to_integer_positions(temp_positions, positions);
        
        LOGD(Default, Info, "Loaded mesh: %d vertices, %d faces", 
             mesh.vertices().size(), mesh.faces().size());
        
        return true;
    }
    catch (const std::exception& e)
    {
        LOGD(Default, Error, "Exception loading mesh %s: %s", path.c_str(), e.what());
        return false;
    }
}

bool EmberApp::save_mesh(const std::string& path, pm::Mesh const& mesh, pm::vertex_attribute<pos_t> const& positions)
{
    try
    {
        // Convert back to double positions for saving
        auto temp_positions = pm::vertex_attribute<tg::dpos3>(mesh);
        convert_to_double_positions(positions, temp_positions);
        
        // Create output directory if needed
        std::filesystem::path file_path(path);
        std::filesystem::create_directories(file_path.parent_path());
        
        pm::save(path, temp_positions);
        
        LOGD(Default, Info, "Saved result mesh: %d vertices, %d faces", 
             mesh.vertices().size(), mesh.faces().size());
        
        return true;
    }
    catch (const std::exception& e)
    {
        LOGD(Default, Error, "Exception saving mesh %s: %s", path.c_str(), e.what());
        return false;
    }
}

void EmberApp::print_usage(const char* program_name)
{
    std::cout << "EMBER - Exact Mesh Boolean Operations\n\n";
    std::cout << "Usage: " << program_name << " [OPTIONS]\n\n";
    std::cout << "Required Arguments:\n";
    std::cout << "  -a, --input-a PATH       Path to first input mesh\n";
    std::cout << "  -b, --input-b PATH       Path to second input mesh\n";
    std::cout << "  -o, --output PATH        Path to output mesh\n\n";
    std::cout << "Optional Arguments:\n";
    std::cout << "  --operation OP           CSG operation: union, intersection, difference [default: union]\n";
    std::cout << "  --format FORMAT          Output format: obj, stl [default: obj]\n";
    std::cout << "  --no-exact               Disable exact arithmetic\n";
    std::cout << "  --no-validate            Disable result validation\n";
    std::cout << "  -h, --help               Show this help message\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << program_name << " -a cube.obj -b sphere.obj -o result.obj --operation union\n";
    std::cout << "  " << program_name << " -a mesh1.stl -b mesh2.stl -o diff.stl --operation difference\n";
}

void EmberApp::convert_to_integer_positions(pm::vertex_attribute<tg::dpos3> const& input_positions,
                                            pm::vertex_attribute<pos_t>& output_positions,
                                            double scaling_factor)
{
    if (scaling_factor <= 0.0)
        scaling_factor = get_scaling_factor(input_positions);
    
    output_positions = pm::vertex_attribute<pos_t>(input_positions.mesh());
    
    for (auto v : input_positions.mesh().vertices())
    {
        auto pos = input_positions[v];
        output_positions[v] = pos_t(
            static_cast<int64_t>(pos.x * scaling_factor),
            static_cast<int64_t>(pos.y * scaling_factor), 
            static_cast<int64_t>(pos.z * scaling_factor)
        );
    }
}

void EmberApp::convert_to_double_positions(pm::vertex_attribute<pos_t> const& input_positions,
                                           pm::vertex_attribute<tg::dpos3>& output_positions,
                                           double scaling_factor)
{
    if (scaling_factor <= 0.0)
        scaling_factor = 1000.0; // Default scaling
    
    output_positions = pm::vertex_attribute<tg::dpos3>(input_positions.mesh());
    
    for (auto v : input_positions.mesh().vertices())
    {
        auto pos = input_positions[v];
        output_positions[v] = tg::dpos3(
            double(pos.x) / scaling_factor,
            double(pos.y) / scaling_factor,
            double(pos.z) / scaling_factor
        );
    }
}

double EmberApp::get_scaling_factor(pm::vertex_attribute<tg::dpos3> const& positions)
{
    double max_coord = 0.0;
    for (auto v : positions.mesh().vertices())
    {
        auto pos = positions[v];
        max_coord = std::max(max_coord, std::abs(pos.x));
        max_coord = std::max(max_coord, std::abs(pos.y));
        max_coord = std::max(max_coord, std::abs(pos.z));
    }
    
    // Choose scaling factor to use reasonable integer range
    // Aim for coordinates in range [-2^20, 2^20] to avoid overflow
    if (max_coord > 0.0)
        return std::min(1000.0, (1 << 20) / max_coord);
    
    return 1000.0; // Default
}

} // namespace mk