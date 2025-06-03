#pragma once

namespace mk
{
struct kernel_options
{
    bool use_unordered_set = false;
    bool use_bb_culling = true;
    int kdop_k = 3;
    bool use_seidel = true;
    bool triangulate = false;
    bool parallel_exact_lp = true;
    int min_faces_for_parallel_setup = 100'000;
};

template <class I>
void introspect(I&& i, kernel_options& v)
{
    i(v.use_unordered_set, "use_unordered_set");
    i(v.use_bb_culling, "use_bb_culling");
    i(v.kdop_k, "kdop_k");
    i(v.use_seidel, "use_seidel");
    i(v.triangulate, "triangulate");
    i(v.parallel_exact_lp, "parallel_exact_lp");
    i(v.min_faces_for_parallel_setup, "min_faces_for_parallel_setup");
}
}
