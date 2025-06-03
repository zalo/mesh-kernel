#include "kernel-app.hh"

// system
#include <cstdio>
#include <filesystem>
#include <string>

// external

#include <clean-core/format.hh>
#include <clean-core/sort.hh>
#include <clean-core/string_view.hh>

#include <imgui/imgui.h>
#include <imgui/imgui_stdlib.h>
#include <imgui/imfilebrowser.hh>

#include <CLI11.hpp>

#include <rich-log/log.hh>

#include <polymesh/algorithms/deduplicate.hh>
#include <polymesh/algorithms/normalize.hh>
#include <polymesh/formats.hh>
#include <polymesh/formats/stl.hh>
#include <polymesh/properties.hh>

#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/canvas.hh>
#include <glow-extras/viewer/experimental.hh>
#include <glow-extras/viewer/view.hh>

#include <integer-plane-geometry/point.hh>

#include <cpp-utils/filesystem.hh>
#include <cpp-utils/rwth_colors.hh>

#include <babel-serializer/data/json.hh>
#include <babel-serializer/file.hh>

#include <ctracer/scope.hh>
#include <ctracer/trace-config.hh>
#include <ctracer/trace.hh>


#if defined(MK_TBB_ENABLED)
#include <tbb/tbb.h>
#endif

// internal
#include <core/kernel-plane-cut.hh>
#include <core/lp-feasibility.hh>

namespace mk
{
RICH_LOG_DEFINE_DEFAULT_DOMAIN("kernel");

KernelApp::KernelApp(rlog::verbosity::type verbosity)
{
    Log::Default::domain.min_verbosity = verbosity;
    m_input_position = pm::vertex_attribute<tg::dpos3>(m_input_mesh);
    m_current_position = pm::vertex_attribute<tg::dpos3>(m_input_mesh);
}

void KernelApp::run(int argc, char** args)
{
    if (argc < 2)
        run_interactive();
    else
        run_cli(argc, args);
}

void KernelApp::init_renderable_set()
{
    //* need to add all groups before creating the references because if the m_renderable_groups gets resized the references might break
    m_renderable_set.get_or_add_renderable_group("input_vertices");
    m_renderable_set.get_or_add_renderable_group("input_edges");
    m_renderable_set.get_or_add_renderable_group("input_faces");

    m_renderable_set.get_or_add_renderable_group("kernel_vertices");
    m_renderable_set.get_or_add_renderable_group("kernel_edges");
    m_renderable_set.get_or_add_renderable_group("kernel_faces");
}


void KernelApp::run_interactive()
{
    glow::glfw::GlfwContext ctx;

    init_renderable_set();

    // (optional) set browser properties
    m_fileDialog.SetTitle("Select mesh");

    gv::interactive(
        [&]()
        {
            handle_imgui();

            m_fileDialog.Display();

            if (m_fileDialog.HasSelected())
            {
                std::cout << "Selected folder " << m_fileDialog.GetSelected().string() << std::endl;
                m_input_directory = m_fileDialog.GetSelected().string();
                fetch_obj_files();
                cc::sort(m_obj_files, [](std::string a, std::string b) { return a < b; });
                m_fileDialog.ClearSelected();
                m_selected_file = m_obj_files[0].c_str();
                update_input_mesh();
            }

            auto v = gv::view();
            v.configure(gv::print_mode, gv::no_grid, gv::no_shadow);

            for (auto const& rg : m_renderable_set.renderable_groups())
            {
                if (!rg.is_enabled)
                    continue;

                for (auto const& r : rg.renderables)
                {
                    gv::view(r);
                }
            }


            if (m_camera_needs_reset)
            {
                m_camera_needs_reset = false;
                gv::experimental::interactive_reset_camera(true);
            }
        });

    exit(0); // end to avoid else nesting
}


void KernelApp::run_cli(int argc, char** args)
{
    CLI::App app{"mesh kernel"};
    bool show_input = false;
    bool show_result = false;
    bool batch_mode = false;
    bool disable_exact_lp = false;
    bool disable_kdop = false;
    bool only_check_exact_feasibility = false;

    std::string input_path;
    std::string output_path;
    std::string output_extension = "obj";

    app.add_option("-i, --input", input_path, "path to input mesh");
    app.add_option("-o, --output", output_path, "path to output mesh");
    app.add_option("-e, --extension", output_extension, "file extension of the output file. Possible values: stl/obj");

    app.add_flag("--disable-exact-lp", disable_exact_lp, "disables the exact linear programming solver to run in parallel an early out check for feasibility");
    app.add_flag("--check-exact-feasibility", only_check_exact_feasibility,
                 "only checks for the existance of a kernel using the exact Seidel solver instead of computing the kernel polyhedron");

    app.add_flag("--show-result", show_result, "renderes the result kernel");
    app.add_flag("--show-input", show_input, "renderes the input mesh");
    app.add_flag("--use-uset", m_options.use_unordered_set, "use unordered set to store cutting planes");

    app.add_flag("--disable-kdop", disable_kdop, "disable the kdop culling");
    app.add_option("-k, --kdop-k", m_options.kdop_k, "sets the kdop k (default = 3, aabb)");
    app.add_flag("--triangulate", m_options.triangulate, "triangulate the output mesh");

    try
    {
        app.parse(argc, args);
    }
    catch (CLI::ParseError const&)
    {
        std::cout << app.help() << std::endl;
        return;
    }

    if (disable_kdop)
        m_options.use_bb_culling = false;

    if (disable_exact_lp)
        m_options.parallel_exact_lp = false;

    if (m_options.triangulate && output_extension == "stl")
    {
        LOGD(Default, Error, "triangulate option is not supported for stl output");
        exit(0);
    }

    output_path = std::filesystem::path(output_path).string();

    util::make_directories(output_path);
    if (!std::filesystem::is_directory(output_path))
    {
        LOGD(Default, Error, "%s must be a valid directory", output_path);
        exit(0);
    }

    std::string const traces_path = std::filesystem::path(output_path + "/traces/").string();
    util::make_directories(traces_path);

    if (!std::filesystem::is_directory(traces_path))
    {
        LOGD(Default, Error, "%s must be a valid directory", traces_path);
        exit(0);
    }


    if (batch_mode)
    {
        run_batch(input_path, output_path, traces_path);
        return;
    }

    //* assert input_path
    if (input_path.length() < 3)
        exit(0);

    LOGD(Default, Info, "Processing %s", input_path);

    if (!load_mesh(input_path, true))
        exit(0);

    if (only_check_exact_feasibility)
    {
        auto feasible = is_feasible(m_input_int_position);
        if (feasible)
        {
            LOGD(Default, Info, "[Feasibility Check]: Has valid kernel!");
        }
        else
        {
            LOGD(Default, Info, "[Feasibility Check]: Kernel is empty!");
        }
        exit(0);
    }

    auto const file_name = std::filesystem::path(input_path).stem().string();

    {
        ct::scope s;
        compute_mesh_kernel();
        ct::write_speedscope_json(s.trace(), traces_path + file_name + ".json");
        babel::file::write(traces_path + file_name + "_metadata.json", babel::json::to_string(m_plane_cut.stats()));
        babel::file::write(traces_path + file_name + "_options.json", babel::json::to_string(m_options));
    }

    LOGD(Default, Info, "done!");

    if (!m_result_empty)
    {
        auto const full_path = output_path + "/" + file_name + "." + output_extension;
        save_kernel(full_path);
    }

    if (show_result || show_input)
    {
        glow::glfw::GlfwContext ctx;
        auto view = gv::view();
        view.configure(gv::print_mode, gv::no_grid);

        if (show_result)
        {
            gv::view(m_current_position, gv::maybe_empty, gv::no_shading, gv::no_shadow);
            gv::view(gv::points(m_current_position).point_size_px(10));
            gv::view(gv::lines(m_current_position).line_width_px(0.5));
        }
        if (show_input)
            gv::view(gv::lines(m_input_position).line_width_px(0.5));
    }
}


void KernelApp::run_batch(std::string const& input_path, std::string const& output_path, std::string const& traces_path)
{
    int total_files = std::distance(std::filesystem::directory_iterator(input_path), std::filesystem::directory_iterator{});
    LOGD(Default, Info, "Total number of files in the directory: %d", total_files);

    int file_count = 0;
    for (auto const& entry : std::filesystem::directory_iterator(input_path))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".obj")
        {
            file_count++;
            auto const input_file = entry.path().string();
            auto const output_file = output_path + "/" + entry.path().filename().string();

            LOGD(Default, Info, "Processing %s/%s file: %s", file_count, total_files, input_file);

            if (!load_mesh(input_file, true))
                continue;

            ct::scope s;

            auto const filen_ame = entry.path().stem().string();

            compute_mesh_kernel();
            ct::write_speedscope_json(s.trace(), traces_path + filen_ame + ".json");

            auto const bm_data = benchmark_data{m_input_mesh.faces().size(), m_current_mesh.faces().size()};
            babel::file::write(traces_path + filen_ame + "_metadata.json", babel::json::to_string(bm_data));

            if (!m_result_empty)
            {
                LOGD(Default, Info, "Writing output to %s", output_file);
                pm::save(output_file, m_current_position);
            }
        }
    }
}

void KernelApp::trace_full_computation()
{
    TRACE("full compute_mesh_kernel");
    compute_mesh_kernel();
}


void KernelApp::save_kernel(cc::string_view filepath)
{
    m_current_mesh.compactify();
    auto path = std::filesystem::path(filepath.begin(), filepath.end());
    LOGD(Default, Info, "Writing output to %s", std::filesystem::absolute(path).string());

    auto const center = tg::dpos3(m_normalize_result.center_x, m_normalize_result.center_y, m_normalize_result.center_z);
    auto tmp_pos = m_current_position.map([&](auto const& p) { return m_normalize_result.scale * p + center; });
    if (filepath.ends_with("stl"))
    {
        auto const pos = tmp_pos.map([](auto const& p) { return tg::pos3(p); });
        auto const face_normals = pm::face_normals(pos);
        auto const pos_array = pos.map([](auto const& p) { return std::array<float, 3>{p.x, p.y, p.z}; });
        auto const face_normals_array = face_normals.map([](auto const& n) { return std::array<float, 3>{n.x, n.y, n.z}; });

        pm::write_stl_binary(path.string(), pos_array, &face_normals_array);
    }
    else
    {
        pm::save(path.string(), tmp_pos);
    }
}

//* sets m_input_mesh and m_input_position

bool KernelApp::load_mesh(cc::string_view const& path, bool normalize)
{
    LOGD(Default, Info, "Loading mesh %s", path);

    m_input_mesh.clear();
    m_input_position.clear();
    if (!pm::load(std::string(path.data(), path.size()), m_input_mesh, m_input_position))
    {
        LOGD(Default, Error, "Failed to load %s", path);
        return false;
    }
    if (m_input_position.empty())
    {
        LOGD(Default, Info, "input mesh %s is empty!", path);
        return false;
    }
    if (!pm::is_closed_mesh(m_input_mesh))
    {
        LOGD(Default, Info, "input mesh %s not closed!", path);
        if (pm::deduplicate(m_input_mesh, m_input_position) == -1)
            return false;
    }

    auto const euler = pm::euler_characteristic(m_input_mesh);
    auto const genus = (2 - euler) * 0.5;
    if (genus > 0)
    {
        LOGD(Default, Info, "input mesh %s has genus > 0!", path);
        return false;
    }

    if (normalize)
        m_normalize_result = pm::normalize(m_input_position);

    m_upscale_factor = get_scaling_factor(m_input_position);
    for (auto const v : m_input_mesh.vertices())
    {
        m_input_int_position[v] = pos_t(m_input_position[v] * m_upscale_factor);
        CC_ASSERT(tg::abs(m_input_int_position[v].x) <= (ipg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(m_input_int_position[v].y) <= (ipg::i64(1) << geometry_t::bits_position));
        CC_ASSERT(tg::abs(m_input_int_position[v].z) <= (ipg::i64(1) << geometry_t::bits_position));
    }

    return true;
}

// returns true if result non-empty

void KernelApp::compute_mesh_kernel()
{
    m_plane_cut.compute_kernel(m_input_int_position, m_options);

    if (!m_plane_cut.has_kernel())
    {
        m_result_empty = true;
        LOGD(Default, Info, "kernel is empty!");
        return;
    }

    m_result_empty = false;

    if (m_plane_cut.input_is_convex())
    {
        LOGD(Default, Info, "Input is convex!");
        m_current_mesh.copy_from(m_input_mesh);
        m_current_position.copy_from(m_input_position);
    }
    else
    {
        auto const& vertex_points = m_plane_cut.position_point4();
        m_current_mesh.copy_from(m_plane_cut.mesh());
        m_current_position = to_dpos(vertex_points.copy_to(m_current_mesh));
        m_current_position.apply([&](tg::dpos3& p) { p = cut_coord_to_normalized_coord(p); });
    }
}

// returns the scaling factor to fit the given points into the integer grid
double KernelApp::get_scaling_factor(pm::vertex_attribute<tg::dpos3> const& points)
{
    auto const& mesh = points.mesh();
    // get max of aabb of points and scale it to the maximum pos representable by integer positions
    auto const aabb = tg::aabb_of(mesh.vertices(), points);
    // check if min is further away from origin than max -> min needs to be scaled
    auto const distance_max_origin = tg::distance_sqr_to_origin(aabb.max);
    auto const distance_min_origin = tg::distance_sqr_to_origin(aabb.min);
    auto const max_point = (distance_max_origin > distance_min_origin) ? aabb.max : tg::abs(aabb.min);
    auto const largest_coordinate = tg::max_element(max_point);

    auto const num_bits = geometry_t::bits_position;
    auto const max_value = (int64_t(1) << num_bits) - 5; // max possible value with num_bits
    float_t const scaling_factor = max_value / largest_coordinate;

    return scaling_factor;
}

pm::vertex_attribute<tg::dpos3> KernelApp::to_dpos(pm::vertex_attribute<point4_t> const& vertex_points)
{
    pm::vertex_attribute<tg::dpos3> result(vertex_points.mesh());
    for (auto vertex_handle : vertex_points.mesh().vertices())
    {
        result(vertex_handle) = ipg::to_dpos3(vertex_points(vertex_handle));
    }
    return result;
}

void KernelApp::fetch_obj_files()
{
    m_obj_files.clear();
    for (auto const& entry : std::filesystem::directory_iterator(m_input_directory))
    {
        auto const extension = entry.path().extension();
        if (entry.is_regular_file() && (extension == ".obj" || extension == ".off" || extension == ".stl"))
        {
            m_obj_files.push_back(entry.path().filename().string());
        }
    }
}


bool KernelApp::select_mesh_window()
{
    if (m_input_directory.empty())
    {
        LOGD(Default, Error, "No input directory set");
        return false;
    }

    ImGui::SetNextWindowPos(ImVec2(300, 60), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(200, 250), ImGuiCond_Once);
    ImGui::Begin("Select Mesh");

    if (ImGui::BeginListBox("##objfiles", ImVec2(-1, 200)))
    {
        for (size_t i = 0; i < m_obj_files.size(); ++i)
        {
            bool is_selected = (m_selected_item == i);
            if (ImGui::Selectable(m_obj_files[i].c_str(), is_selected))
            {
                m_selected_item = i;
                m_selected_file = m_obj_files[i].c_str(); // Update the selected file
            }
        }
        ImGui::EndListBox();
    }

    // Confirm button
    if (ImGui::Button("Confirm Selection") && !m_selected_file.empty())
    {
        ImGui::End();
        update_input_mesh();
        return false;
    }
    if (ImGui::Button("Close"))
    {
        ImGui::End();
        return false;
    }

    ImGui::End();
    return true;
}


void KernelApp::update_input_mesh()
{
    auto const input_file_path = cc::format("%s/%s", m_input_directory, m_selected_file);
    if (!std::ifstream(input_file_path.c_str()))
    {
        LOGD(Default, Error, "File %s does not exist", input_file_path);
        return;
    }
    load_mesh(input_file_path);
    add_renderable_to_groups("input", m_input_position);

    m_camera_needs_reset = true;
    m_loaded_file = m_selected_file;
    reset_renderable_goup("kernel");
}


void KernelApp::reset_renderable_goup(cc::string_view name)
{
    auto& vertex_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_vertices").c_str());
    auto& edge_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_edges").c_str());
    auto& face_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_faces").c_str());

    vertex_group.renderables.clear();
    edge_group.renderables.clear();
    face_group.renderables.clear();
}


void KernelApp::handle_imgui()
{
    auto& input_vertex_group = m_renderable_set.get_or_add_renderable_group("input_vertices");
    auto& input_edge_group = m_renderable_set.get_or_add_renderable_group("input_edges");
    auto& input_face_group = m_renderable_set.get_or_add_renderable_group("input_faces");

    auto& kernel_vertex_group = m_renderable_set.get_or_add_renderable_group("kernel_vertices");
    auto& kernel_edge_group = m_renderable_set.get_or_add_renderable_group("kernel_edges");
    auto& kernel_face_group = m_renderable_set.get_or_add_renderable_group("kernel_faces");

    ImGui::Begin("mesh kernel");

    ImGui::Separator();
    ImGui::SeparatorText("Input options");
    if (ImGui::Button("Select Dir"))
    {
        m_fileDialog.Open();
    }
    ImGui::Text("Selected directory: %s", m_input_directory.c_str());
    ImGui::Text("Loaded file: %s", m_loaded_file.c_str());
    ImGui::Checkbox("show input vertices", &input_vertex_group.is_enabled);
    ImGui::Checkbox("show input edges", &input_edge_group.is_enabled);
    ImGui::Checkbox("show input faces", &input_face_group.is_enabled);

    // Button to open the new window
    if (ImGui::Button("Select Input Mesh") || m_show_select_mesh_window)
    {
        m_show_select_mesh_window = select_mesh_window();
    }

    handle_key_events();


    ImGui::Separator();
    ImGui::SeparatorText("Kernel computation options");
    ImGui::Checkbox("use unordered set to store planes", &m_options.use_unordered_set);
    ImGui::Checkbox("use bounding box culling", &m_options.use_bb_culling);
    ImGui::Checkbox("use seidel solver to early out", &m_options.use_seidel);
    char const* kdop_options[] = {"3", "8", "9", "12"};
    static int kdop_current = 0; // Default to first option

    if (ImGui::BeginCombo("kdop k", kdop_options[kdop_current]))
    {
        for (int n = 0; n < IM_ARRAYSIZE(kdop_options); n++)
        {
            bool is_selected = (kdop_current == n);
            if (ImGui::Selectable(kdop_options[n], is_selected))
            {
                kdop_current = n;
                m_options.kdop_k = std::stoi(kdop_options[n]);
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::Separator();
    ImGui::SeparatorText("Result options");
    ImGui::Checkbox("show kernel vertices", &kernel_vertex_group.is_enabled);
    ImGui::Checkbox("show kernel edges", &kernel_edge_group.is_enabled);
    ImGui::Checkbox("show kernel faces", &kernel_face_group.is_enabled);

    if (ImGui::Button("Compute Kernel"))
    {
        if (!m_input_position.empty())
        {
            compute_mesh_kernel();
            ct::write_speedscope_json("kernel.json");
            if (!m_result_empty)
            {
                add_renderable_to_groups("kernel", m_current_position);
            }
            m_pop_up_shown = false;
        }
    }

    ImGui::BeginDisabled(m_result_empty);

    ImGui::InputText("##output_filepath", &m_output_file);
    ImGui::SameLine();
    if (ImGui::Button("Save Kernel"))
    {
        if (util::exists(m_output_file))
        {
            ImGui::OpenPopup("File exists");
        }
        else
        {
            save_kernel(m_output_file);
        }
    }
    ImGui::EndDisabled();

    if (ImGui::BeginPopup("File exists"))
    {
        ImGui::Text("File already exists! Overwrite?");
        ImGui::Text("%s", m_output_file.c_str());
        if (ImGui::Button("Cancel"))
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Overwrite"))
        {
            save_kernel(m_output_file);
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (m_result_empty && !m_pop_up_shown)
    {
        ImGui::OpenPopup("Popup");
    }

    // Define the pop-up modal
    if (ImGui::BeginPopupModal("Popup", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Kernel is empty!");

        if (ImGui::Button("Close"))
        {
            ImGui::CloseCurrentPopup(); // Close the pop-up when the button is clicked
            m_pop_up_shown = true;
        }

        ImGui::EndPopup();
    }

    ImGui::End();
}


void KernelApp::add_renderable_to_groups(cc::string_view name, pm::vertex_attribute<tg::dpos3> const& positions)
{
    auto& vertex_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_vertices").c_str());
    auto& edge_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_edges").c_str());
    auto& face_group = m_renderable_set.get_or_add_renderable_group(cc::format("%s%s", name, "_faces").c_str());

    tg::color3 vertex_color;
    tg::color3 edge_color;
    tg::color3 face_color;

    if (name == "input")
    {
        vertex_color = util::rwth::petrol_100;
        edge_color = util::rwth::petrol_75;
        face_color = util::rwth::petrol_50;
    }
    if (name == "kernel")
    {
        vertex_color = util::rwth::may_green_100;
        edge_color = util::rwth::may_green_75;
        face_color = util::rwth::may_green_50;
    }

    auto const aabb = aabb_of(positions);
    auto const diag_length = tg::length(aabb.max - aabb.min);
    auto const line_width = 0.001 * diag_length;
    auto const point_size = 0.001 * diag_length;

    gv::canvas_data canvas_data;
    canvas_data.set_point_size_world(point_size);
    canvas_data.add_points(positions).color(vertex_color);
    vertex_group.renderables = cc::vector<gv::SharedRenderable>(canvas_data.create_renderables());
    canvas_data.clear();

    canvas_data.set_line_width_world(line_width);
    canvas_data.add_lines(positions).color(edge_color);
    edge_group.renderables = cc::vector<gv::SharedRenderable>(canvas_data.create_renderables());
    canvas_data.clear();

    canvas_data.add_faces(positions).color(face_color);
    face_group.renderables = cc::vector<gv::SharedRenderable>(canvas_data.create_renderables());
}


void KernelApp::handle_key_events()
{
    if (m_input_directory.empty())
        return;
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow))
    {
        if (m_selected_item == m_obj_files.size() - 1)
            m_selected_item = 0;
        else
        {
            m_selected_item++;
        }
        m_selected_file = m_obj_files[m_selected_item].c_str();
        update_input_mesh();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))
    {
        if (m_selected_item == 0)
            m_selected_item = m_obj_files.size() - 1;
        else
        {
            m_selected_item--;
        }
        m_selected_file = m_obj_files[m_selected_item].c_str();
        update_input_mesh();
    }
}
} // namespace mk
