/*!
 * @brief Plugin definition
 * @file idssePlugin.hpp
 */
#ifndef EZC2X_IDSSE_PLUGIN_HPP
#define EZC2X_IDSSE_PLUGIN_HPP

#include <ezC2X/core/plugin/Plugin.hpp>

namespace ezC2X
{

/*!
 * @brief The plugin class
 */
class idssePlugin : public Plugin
{
public:
    /*
     * Plugin interface
     */

    PluginInfo const
    getPluginInfo() const override;

    void
    populateTypeSet(TypeSet& ts) override;
};

EXPORT_EZC2X_PLUGIN(idssePlugin)

} // namespace ezC2X

#endif // EZC2X_IDSSE_PLUGIN_HPP
