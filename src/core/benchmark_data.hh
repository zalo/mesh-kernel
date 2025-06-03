#pragma once

namespace mk
{
struct benchmark_data
{
    int input_faces = 0;
    int kernel_faces = 0;
    int convex_contribution_kernel = 0;
    int concave_contribution_kernel = 0;
    bool is_convex = false;
    bool lp_early_out = false;
    int number_concave_planes = 0;
    int total_planes = 0;

    double time_plane_orracle_seconds = 0.0;
};

template <class I>
void introspect(I&& i, benchmark_data& data)
{
    i(data.input_faces, "input_faces");
    i(data.kernel_faces, "kernel_faces");
    i(data.convex_contribution_kernel, "convex_contribution_kernel");
    i(data.concave_contribution_kernel, "concave_contribution_kernel");
    i(data.is_convex, "is_convex");
    i(data.lp_early_out, "lp_early_out");
    i(data.number_concave_planes, "number_concave_planes");
    i(data.total_planes, "total_planes");
    i(data.time_plane_orracle_seconds, "time_plane_orracle_seconds");
}
}
