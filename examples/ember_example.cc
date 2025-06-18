// Example: Using EMBER for CSG operations
#include <core/ember-csg.hh>
#include <core/ember-app.hh>
#include <polymesh/objects/cube.hh>
#include <polymesh/objects/sphere.hh>
#include <polymesh/formats.hh>
#include <iostream>

using namespace mk;

int main()
{
    std::cout << "EMBER CSG Example\n";
    std::cout << "=================\n\n";

    // Create two simple meshes for demonstration
    pm::Mesh cube_mesh, sphere_mesh;
    
    // Create a cube
    auto cube_pos = pm::vertex_attribute<tg::dpos3>(cube_mesh);
    pm::objects::add_cube(cube_mesh, cube_pos);
    
    // Create a sphere (using cube as placeholder)
    sphere_mesh = cube_mesh;
    auto sphere_pos = pm::vertex_attribute<tg::dpos3>(sphere_mesh);
    for (auto v : sphere_mesh.vertices())
    {
        sphere_pos[v] = cube_pos[pm::vertex_handle(v.idx.value)] * 0.7; // Smaller
    }
    
    std::cout << "Created cube with " << cube_mesh.vertices().size() << " vertices\n";
    std::cout << "Created sphere with " << sphere_mesh.vertices().size() << " vertices\n\n";
    
    // Convert to EMBER integer positions
    EmberCSG csg_processor;
    
    auto cube_int_pos = pm::vertex_attribute<EmberCSG::pos_t>(cube_mesh);
    auto sphere_int_pos = pm::vertex_attribute<EmberCSG::pos_t>(sphere_mesh);
    
    // Convert coordinates to integer (scale by 1000)
    for (auto v : cube_mesh.vertices())
    {
        auto pos = cube_pos[v];
        cube_int_pos[v] = EmberCSG::pos_t(
            static_cast<int64_t>(pos.x * 1000),
            static_cast<int64_t>(pos.y * 1000),
            static_cast<int64_t>(pos.z * 1000)
        );
    }
    
    for (auto v : sphere_mesh.vertices())
    {
        auto pos = sphere_pos[v];
        sphere_int_pos[v] = EmberCSG::pos_t(
            static_cast<int64_t>(pos.x * 1000),
            static_cast<int64_t>(pos.y * 1000),
            static_cast<int64_t>(pos.z * 1000)
        );
    }
    
    // Set up EMBER options
    ember_options options;
    options.use_exact_arithmetic = true;
    options.validate_results = true;
    
    // Perform union operation
    std::cout << "Computing union...\n";
    bool union_success = csg_processor.compute_union(cube_int_pos, sphere_int_pos, options);
    
    if (union_success)
    {
        std::cout << "Union successful! Result has " 
                  << csg_processor.result_mesh().vertices().size() << " vertices and "
                  << csg_processor.result_mesh().faces().size() << " faces\n";
        
        // Save result (would need proper coordinate conversion back to double)
        // pm::save("union_result.obj", csg_processor.result_mesh(), result_double_positions);
    }
    else
    {
        std::cout << "Union failed!\n";
    }
    
    // Perform intersection operation
    std::cout << "\nComputing intersection...\n";
    bool intersection_success = csg_processor.compute_intersection(cube_int_pos, sphere_int_pos, options);
    
    if (intersection_success)
    {
        std::cout << "Intersection successful! Result has "
                  << csg_processor.result_mesh().vertices().size() << " vertices and "
                  << csg_processor.result_mesh().faces().size() << " faces\n";
    }
    else
    {
        std::cout << "Intersection failed!\n";
    }
    
    // Perform difference operation  
    std::cout << "\nComputing difference (cube - sphere)...\n";
    bool difference_success = csg_processor.compute_difference(cube_int_pos, sphere_int_pos, options);
    
    if (difference_success)
    {
        std::cout << "Difference successful! Result has "
                  << csg_processor.result_mesh().vertices().size() << " vertices and "
                  << csg_processor.result_mesh().faces().size() << " faces\n";
    }
    else
    {
        std::cout << "Difference failed!\n";
    }
    
    std::cout << "\nEMBER CSG example completed!\n";
    
    return (union_success && intersection_success && difference_success) ? 0 : 1;
}