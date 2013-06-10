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
	connect(ui_.board_min_size_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.board_canny_lower_threshold_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));

	connect(ui_.puck_hue_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_hue_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_sat_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_low_spinBox,  SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_val_high_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.encircle_min_size_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.encircle_max_size_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puckiness_min_ratio_doubleSpinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.puck_canny_lower_threshold_spinBox, SIGNAL(editingFinished(void)), this, SLOT(onEditingFinished(void)));
	connect(ui_.find_pucks_pushButton,  SIGNAL(clicked(void)), this, SLOT(find_pucks(void)));
	connect(ui_.save_params_pushButton, SIGNAL(clicked(void)), this, SLOT(save_parameters(void)));
	connect(ui_.load_params_pushButton, SIGNAL(clicked(void)), this, SLOT(load_parameters(void)));

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

	cb_params_msg.find_pucks = 0;
}

void CBGUI::onEditingFinished(void)
{
	cb_params_msg.board_hue_low  = ui_.board_hue_low_spinBox->value();
	cb_params_msg.board_hue_high = ui_.board_hue_high_spinBox->value();
	cb_params_msg.board_sat_low  = ui_.board_sat_low_spinBox->value();
	cb_params_msg.board_sat_high = ui_.board_sat_high_spinBox->value();
	cb_params_msg.board_val_low  = ui_.board_val_low_spinBox->value();
	cb_params_msg.board_val_high = ui_.board_val_high_spinBox->value();
	cb_params_msg.board_min_size = ui_.board_min_size_spinBox->value();
	cb_params_msg.board_canny_lower_threshold = ui_.board_canny_lower_threshold_spinBox->value();

	cb_params_msg.puck_hue_low  = ui_.puck_hue_low_spinBox->value();
	cb_params_msg.puck_hue_high = ui_.puck_hue_high_spinBox->value();
	cb_params_msg.puck_sat_low  = ui_.puck_sat_low_spinBox->value();
	cb_params_msg.puck_sat_high = ui_.puck_sat_high_spinBox->value();
	cb_params_msg.puck_val_low  = ui_.puck_val_low_spinBox->value();
	cb_params_msg.puck_val_high = ui_.puck_val_high_spinBox->value();
	cb_params_msg.encircle_min_size = ui_.encircle_min_size_spinBox->value();
	cb_params_msg.encircle_max_size = ui_.encircle_max_size_spinBox->value();
	cb_params_msg.puckiness_min_ratio = ui_.puckiness_min_ratio_doubleSpinBox->value();
	cb_params_msg.puck_canny_lower_threshold = ui_.puck_canny_lower_threshold_spinBox->value();
}

void CBGUI::find_pucks(void)
{
	cb_params_msg.find_pucks = 20;   // TODO: Maybe make this more configurable?
}

void CBGUI::save_parameters(void)
{
	set_parameters();

	int rosparamPID = fork();
	if (rosparamPID == 0) {   // Child process
		execlp("rosparam", "rosparam", "dump", "/opt/ros/workspace/engr421/cb_vision/gui_config.yaml", "/cb_board", NULL);
		exit(127);   // Exit code 127 if command not found.
	}

	ROS_INFO("GUI: Saved GUI settings.");
}

void CBGUI::load_parameters(void)
{
	int rosparamPID = fork();
	if (rosparamPID == 0) {   // Child process
		execlp("rosparam", "rosparam", "load", "/opt/ros/workspace/engr421/cb_vision/gui_config.yaml", "/cb_board", NULL);
		exit(127);   // Exit code 127 if command not found.
	}

	// Once parameters are loaded, get them and update GUI.
	while(nh_.hasParam("cb_board"));

	get_parameters();

	ROS_INFO("GUI: Loaded GUI settings.");
}

void CBGUI::set_parameters(void)
{
	nh_.setParam("/cb_board/puck_hue_low", cb_params_msg.puck_hue_low);
	nh_.setParam("/cb_board/puck_hue_high", cb_params_msg.puck_hue_high);
	nh_.setParam("/cb_board/puck_sat_low", cb_params_msg.puck_sat_low);
	nh_.setParam("/cb_board/puck_sat_high", cb_params_msg.puck_sat_high);
	nh_.setParam("/cb_board/puck_val_low", cb_params_msg.puck_val_low);
	nh_.setParam("/cb_board/puck_val_high", cb_params_msg.puck_val_high);
	nh_.setParam("/cb_board/encircle_min_size", cb_params_msg.encircle_min_size);
	nh_.setParam("/cb_board/encircle_max_size", cb_params_msg.encircle_max_size);
	nh_.setParam("/cb_board/puckiness_min_ratio", cb_params_msg.puckiness_min_ratio);
	nh_.setParam("/cb_board/puck_canny_lower_threshold", cb_params_msg.puck_canny_lower_threshold);
}

void CBGUI::get_parameters(void)
{
	// Temporary variables of ROS-compatible types.
	static int puck_hue_low;
	static int puck_hue_high;
	static int puck_sat_low;
	static int puck_sat_high;
	static int puck_val_low;
	static int puck_val_high;
	static int encircle_min_size;
	static int encircle_max_size;
	static int puck_erosion_iter;
	static double puckiness_min_ratio;
	static int puck_canny_lower_threshold;

	// Get parameters.
	nh_.getParam("/cb_board/puck_hue_low",               puck_hue_low);
	nh_.getParam("/cb_board/puck_hue_high",              puck_hue_high);
	nh_.getParam("/cb_board/puck_sat_low",               puck_sat_low);
	nh_.getParam("/cb_board/puck_sat_high",              puck_sat_high);
	nh_.getParam("/cb_board/puck_val_low",               puck_val_low);
	nh_.getParam("/cb_board/puck_val_high",              puck_val_high);
	nh_.getParam("/cb_board/encircle_min_size",          encircle_min_size);
	nh_.getParam("/cb_board/encircle_max_size",          encircle_max_size);
	nh_.getParam("/cb_board/puck_erosion_iter",          puck_erosion_iter);
	nh_.getParam("/cb_board/puckiness_min_ratio",        puckiness_min_ratio);
	nh_.getParam("/cb_board/puck_canny_lower_threshold", puck_canny_lower_threshold);

	// Set GUI fields.
	ui_.puck_hue_low_spinBox->setValue(puck_hue_low);
	ui_.puck_hue_high_spinBox->setValue(puck_hue_high);
	ui_.puck_sat_low_spinBox->setValue(puck_sat_low);
	ui_.puck_sat_high_spinBox->setValue(puck_sat_high);
	ui_.puck_val_low_spinBox->setValue(puck_val_low);
	ui_.puck_val_high_spinBox->setValue(puck_val_high);
	ui_.encircle_min_size_spinBox->setValue(encircle_min_size);
	ui_.encircle_max_size_spinBox->setValue(encircle_max_size);
	ui_.puckiness_min_ratio_doubleSpinBox->setValue(puckiness_min_ratio);
	ui_.puck_canny_lower_threshold_spinBox->setValue(puck_canny_lower_threshold);
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
