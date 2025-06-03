#include "renderable_set.hh"

#include <clean-core/span.hh>
#include <clean-core/string.hh>
#include <clean-core/string_view.hh>
#include <clean-core/vector.hh>

#include <glow-extras/viewer/renderables/Renderable.hh>
#include <glow-extras/viewer/view.hh>

// gives an existing renderable group with the same name or a new one
mk::RenderableSet::renderable_group& mk::RenderableSet::get_or_add_renderable_group(cc::string_view name)
{
    for (auto& rg : m_renderable_groups)
    {
        if (rg.name == name)
        {
            return rg;
        }
    }
    return m_renderable_groups.emplace_back(renderable_group{name, {}, true});
}

void mk::RenderableSet::add_renderable_group(cc::string_view name, cc::span<glow::viewer::SharedRenderable> renderables)
{
    auto& rg = get_or_add_renderable_group(name);
    rg.renderables.clear();
    rg.renderables.push_back_range(renderables);
    rg.is_enabled = true;
}

void mk::RenderableSet::add_renderable_group(cc::string_view name, glow::viewer::SharedRenderable renderable)
{
    add_renderable_group(name, cc::span{&renderable, 1});
}

// cc::vector<gv::SharedRenderable const&> RenderableSet::get_active_renderables()
// {
//     cc::vector<gv::SharedRenderable const&> all_renderables;

//     for (auto const& rg : m_renderable_groups)
//     {
//         if (rg.is_enabled)
//         {
//             for (auto& r : rg.renderables)
//             {
//                 all_renderables.push_back(r);
//             }
//         }
//     }

//     return all_renderables;
// }
