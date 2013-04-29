#include "cb_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_cb_gui {

CBGUI::CBGUI()
	: rqt_gui_cpp::Plugin()
	, widget_(0)
{
	// Constructor is called first before initPlugin function, needless to say.

	// give QObjects reasonable names
	setObjectName("CBGUI");
}

void CBGUI::initPlugin(qt_gui_cpp::PluginContext& context)
{
	// access standalone command line arguments
	QStringList argv = context.argv();
	// create QWidget
	widget_ = new QWidget();
	// extend the widget with all attributes and children from UI file
	ui_.setupUi(widget_);
	// add widget to the user interface
	context.addWidget(widget_);

	// Get nodehandle and advertise publisher.
	nh_ = getNodeHandle();
	cb_gui_pub = nh_.advertise<rqt_cb_gui::cb_params>("cb_params", 1);

	// Connect ROS timer to publisher callback.
	pub_timer = nh_.createTimer(ros::Duration(0.02), &CBGUI::pub_cb, this);

	// Connect UI elements to callback for updating cb_params_msg. Be sure to
	// include every element that can be updated here!
	connect(ui_.puck_hue_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_hue_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
}

void CBGUI::shutdownPlugin()
{
	// TODO unregister all publishers here
}

void CBGUI::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& 
instance_settings) const
{
	// TODO save intrinsic configuration, usually using:
	// instance_settings.setValue(k, v)
}

void CBGUI::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& 
instance_settings)
{
	// TODO restore intrinsic configuration, usually using:
	// v = instance_settings.value(k)
}

void CBGUI::pub_cb(const ros::TimerEvent& event)
{
	cb_gui_pub.publish(cb_params_msg);
}

void CBGUI::onEditingFinished(void)
{
	cb_params_msg.puck_hue_low  = ui_.puck_hue_low_spinBox->value();
	cb_params_msg.puck_hue_high = ui_.puck_hue_high_spinBox->value();
	cb_params_msg.puck_sat_low  = ui_.puck_sat_low_spinBox->value();
	cb_params_msg.puck_sat_high = ui_.puck_sat_high_spinBox->value();
	cb_params_msg.puck_val_low  = ui_.puck_val_low_spinBox->value();
	cb_params_msg.puck_val_high = ui_.puck_val_high_spinBox->value();
}

/*bool hasConfiguration() const
{
	return true;
}

void triggerConfiguration()
{
	// Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_cb_gui, CBGUI, rqt_cb_gui::CBGUI, rqt_gui_cpp::Plugin)
