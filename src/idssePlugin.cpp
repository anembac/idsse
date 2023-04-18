#include "idssePlugin.hpp"

#include <boost/preprocessor/stringize.hpp>

#include <ezC2X/core/component/Create.hpp>
#include <ezC2X/framework/ApplicationFactory.hpp>

#include "idsse.hpp"

namespace ezC2X
{
PluginInfo const
idssePlugin::getPluginInfo() const
{
    return PluginInfo{"idssePlugin", BOOST_PP_STRINGIZE(PLUGIN_VERSION)};
}

void
idssePlugin::populateTypeSet(TypeSet& ts)
{
    // register application at the factory
    auto& f = ts.get<ApplicationFactory>();
    f.add("idsse",
          [](auto const& p) { return ezC2X::component::create<idsse>(p); });
}

}  // namespace ezC2X
