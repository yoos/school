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
	// TODO: Functionify this.
	connect(ui_.board_hue_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_hue_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_sat_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_sat_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_val_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_val_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_erosion_iter_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_dilation_iter_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_canny_lower_threshold_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));

	connect(ui_.puck_hue_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_hue_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.encircle_min_size_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.encircle_max_size_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_erosion_iter_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puckiness_min_ratio_doubleSpinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_canny_lower_threshold_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));

	// Call editingFinished() once at beginning to get the values out there.
	onEditingFinished();
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
	cb_params_msg.board_hue_low  = ui_.board_hue_low_spinBox->value();
	cb_params_msg.board_hue_high = ui_.board_hue_high_spinBox->value();
	cb_params_msg.board_sat_low  = ui_.board_sat_low_spinBox->value();
	cb_params_msg.board_sat_high = ui_.board_sat_high_spinBox->value();
	cb_params_msg.board_val_low  = ui_.board_val_low_spinBox->value();
	cb_params_msg.board_val_high = ui_.board_val_high_spinBox->value();
	cb_params_msg.board_erosion_iter = ui_.board_erosion_iter_spinBox->value();
	cb_params_msg.board_dilation_iter = ui_.board_dilation_iter_spinBox->value();
	cb_params_msg.board_canny_lower_threshold = ui_.board_canny_lower_threshold_spinBox->value();

	cb_params_msg.puck_hue_low  = ui_.puck_hue_low_spinBox->value();
	cb_params_msg.puck_hue_high = ui_.puck_hue_high_spinBox->value();
	cb_params_msg.puck_sat_low  = ui_.puck_sat_low_spinBox->value();
	cb_params_msg.puck_sat_high = ui_.puck_sat_high_spinBox->value();
	cb_params_msg.puck_val_low  = ui_.puck_val_low_spinBox->value();
	cb_params_msg.puck_val_high = ui_.puck_val_high_spinBox->value();
	cb_params_msg.encircle_min_size = ui_.encircle_min_size_spinBox->value();
	cb_params_msg.encircle_max_size = ui_.encircle_max_size_spinBox->value();
	cb_params_msg.puck_erosion_iter  = ui_.puck_erosion_iter_spinBox->value();
	cb_params_msg.puckiness_min_ratio = ui_.puckiness_min_ratio_doubleSpinBox->value();
	cb_params_msg.puck_canny_lower_threshold = ui_.puck_canny_lower_threshold_spinBox->value();
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
