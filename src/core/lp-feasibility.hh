#pragma once

#include <rich-log/log.hh>

#include <polymesh/Mesh.hh>

#include <typed-geometry/tg.hh>

namespace mk
{
bool is_feasible(pm::vertex_attribute<tg::ipos3> const& positions);
} // namespace mk
