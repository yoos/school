#ifndef rqt_cb_gui__CBGUI_H
#define rqt_cb_gui__CBGUI_H

#include <ros/ros.h>

#include <rqt_gui_cpp/plugin.h>
#include <rqt_cb_gui/ui_cb_gui.h>
#include <QWidget>

#include <rqt_cb_gui/cb_params.h>

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
	virtual void pub_cb(void);   // Publisher callback.

	// Comment in to signal that the plugin has a way to configure it
	//bool hasConfiguration() const;
	//void triggerConfiguration();
private:
	Ui::CBGUIWidget ui_;
	QWidget* widget_;

	ros::NodeHandle nh_;   // Private nodehandle.
	rqt_cb_gui::cb_params cb_params_msg;
	ros::Publisher cb_gui_pub;
};
} // namespace
#endif // rqt_cb_gui__CBGUI_H
