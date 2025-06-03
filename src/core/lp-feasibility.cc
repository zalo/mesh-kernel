
#include "lp-feasibility.hh"

#include <chrono>

#include <rich-log/log.hh>

#include <polymesh/Mesh.hh>

#include <typed-geometry/tg.hh>

// internal
#include <core/ExactSeidelSolverPoint.hh>

bool mk::is_feasible(pm::vertex_attribute<tg::ipos3> const& positions)
{
    using geometry_t = ipg::geometry<26, 55>;
    using plane_t = typename geometry_t::plane_t;

    // todo: take planes without duplicates
    cc::vector<plane_t> planes;
    for (auto const f : positions.mesh().faces())
    {
        auto const pts = f.vertices().to_array<3>(positions);
        auto const p = plane_t::from_points_no_gcd(pts[0], pts[1], pts[2]);
        if (tg::is_zero(p.a) && tg::is_zero(p.b) && tg::is_zero(p.c))
            continue;
        planes.push_back(p);
    }

    ExactSeidelSolverPoint solver;
    solver.set_planes(planes);
    auto t0 = std::chrono::high_resolution_clock::now();
    auto state = solver.solve();
    auto t1 = std::chrono::high_resolution_clock::now();

    auto const elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

    LOGD(Default, Info, "Feasibility check took {}ns using exact seidel", elapsed_ns);

    return state != ExactSeidelSolverPoint::state::infeasible;
}
