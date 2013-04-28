#ifndef rqt_cb_gui__CBGUI_H
#define rqt_cb_gui__CBGUI_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_cb_gui/ui_cb_gui.h>
#include <QWidget>

namespace rqt_cb_gui {

class CBGUI
	: public rqt_gui_cpp::Plugin
{
	Q_OBJECT
public:
	CBGUI();
	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

	// Comment in to signal that the plugin has a way to configure it
	//bool hasConfiguration() const;
	//void triggerConfiguration();
private:
	Ui::CBGUIWidget ui_;
	QWidget* widget_;
};
} // namespace
#endif // rqt_cb_gui__CBGUI_H
