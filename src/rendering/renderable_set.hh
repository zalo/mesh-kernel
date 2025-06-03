#include <clean-core/span.hh>
#include <clean-core/string.hh>
#include <clean-core/string_view.hh>
#include <clean-core/vector.hh>

#include <glow-extras/viewer/renderables/Renderable.hh>

namespace mk
{
/// container for renderables
/// multiple renderables can be joined together into a single renderable_group
/// each group is identified by a unique name and can be enabled/disabled
class RenderableSet
{
public: // types
    /// a single renderable group
    struct renderable_group
    {
        cc::string name;
        cc::vector<glow::viewer::SharedRenderable> renderables;
        bool is_enabled = true;
    };

public: // adding renderables
    /// gives an existing renderable group with the same name or a new one
    renderable_group& get_or_add_renderable_group(cc::string_view name);

    /// create a new renderable_group with the given name and renderables
    /// if a group with the same name already exists, it is overwritten
    void add_renderable_group(cc::string_view name, cc::span<glow::viewer::SharedRenderable> renderables);

    /// same as above, but with a single renderable
    void add_renderable_group(cc::string_view name, glow::viewer::SharedRenderable renderable);

    // cc::vector<glow::viewer::SharedRenderable const&> get_active_renderables();

public: // getter
    cc::span<renderable_group> renderable_groups() { return m_renderable_groups; }
    cc::span<renderable_group const> renderable_groups() const { return m_renderable_groups; }

private:
    cc::vector<renderable_group> m_renderable_groups;
};
}
