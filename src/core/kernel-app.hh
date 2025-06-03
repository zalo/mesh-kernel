#pragma once

#include <polymesh/Mesh.hh>
#include <polymesh/algorithms/normalize.hh>

#include <typed-geometry/tg-lean.hh>

#include <imgui/imgui.h>
#include <imgui/imfilebrowser.hh>

#include <rich-log/log.hh>

#include <clean-core/string_view.hh>

#include <integer-plane-geometry/geometry.hh>

// internal
#include <core/kernel-plane-cut.hh>
#include <rendering/renderable_set.hh>

namespace mk
{

RICH_LOG_DECLARE_DEFAULT_DOMAIN();

class KernelApp
{
public: // types
    using geometry_t = ipg::geometry<26, 55>;
    using pos_t = typename geometry_t::pos_t;
    using vec_t = typename geometry_t::vec_t;
    using point4_t = typename geometry_t::point4_t;
    using plane_t = typename geometry_t::plane_t;

public:
    KernelApp(rlog::verbosity::type verbosity = rlog::verbosity::Info);

    void run(int argc, char** args);

    double get_scaling_factor(pm::vertex_attribute<tg::dpos3> const& points);

private: // members
    pm::Mesh m_input_mesh;
    pm::vertex_attribute<tg::dpos3> m_input_position{m_input_mesh};
    pm::vertex_attribute<tg::ipos3> m_input_int_position{m_input_mesh};
    pm::normalize_result<double> m_normalize_result;

    pm::Mesh m_current_mesh;
    pm::vertex_attribute<tg::dpos3> m_current_position{m_current_mesh};
    kernel_options m_options;

    bool m_result_empty = true;

    double m_upscale_factor = 0.0f;

    KernelPlaneCut m_plane_cut;

private: // gui
    std::string m_input_directory;
    std::string m_output_directory;
    cc::string m_selected_file;
    cc::string m_loaded_file;
    cc::vector<std::string> m_obj_files;
    size_t m_selected_item = 0;

    bool m_show_select_mesh_window = false;
    bool m_pop_up_shown = true;

    bool m_camera_needs_reset = false;
    ImGui::FileBrowser m_fileDialog{ImGuiFileBrowserFlags_SelectDirectory};
    std::string m_output_file = "kernel.stl"; // interactive mode only

    RenderableSet m_renderable_set;

private: // helper
    tg::dpos3 cut_coord_to_normalized_coord(tg::dpos3 const& p) { return p / m_upscale_factor; }
    tg::dpos3 normalized_coord_to_cut_coord(tg::dpos3 const& p) { return p * m_upscale_factor; }

    void run_interactive();

    void run_cli(int argc, char** args);

    void run_batch(std::string const& input_path, std::string const& output_path, std::string const& traces_path);

    bool load_mesh(cc::string_view const& path, bool normalize = true);

    void compute_mesh_kernel();

    pm::vertex_attribute<tg::dpos3> to_dpos(pm::vertex_attribute<point4_t> const& vertex_points);

    void handle_imgui();

    bool select_mesh_window();

    void fetch_obj_files();

    void update_input_mesh();

    void init_renderable_set();

    void add_renderable_to_groups(cc::string_view name, pm::vertex_attribute<tg::dpos3> const& positions);

    void reset_renderable_goup(cc::string_view name);

    void handle_key_events();

    void trace_full_computation();

    void save_kernel(cc::string_view filepath);
};

} // namespace mk
