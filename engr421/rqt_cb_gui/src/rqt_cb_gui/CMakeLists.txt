find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

set(rqt_cb_gui_SRCS
	cb_gui.cpp
)

set(rqt_cb_gui_HDRS
	cb_gui.h
)

set(rqt_cb_gui_UIS
	cb_gui.ui
)

set(rqt_cb_gui_INCLUDE_DIRECTORIES
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/build/src
)

qt4_wrap_cpp(rqt_cb_gui_MOCS ${rqt_cb_gui_HDRS} OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)   # Boost 1.53 and Qt4 MOC don't play nicely without this options definition. See https://bugreports.qt-project.org/browse/QTBUG-22829.
qt4_wrap_ui(rqt_cb_gui_UIS_H ${rqt_cb_gui_UIS})

#rosbuild_include(pluginlib hide_plugin_symbols)
#pluginlib_hide_plugin_symbols(rqt_cb_gui CBGUI)

include_directories(${PROJECT_NAME} ${rqt_cb_gui_INCLUDE_DIRECTORIES})
rosbuild_add_library(${PROJECT_NAME} ${rqt_cb_gui_SRCS} ${rqt_cb_gui_MOCS} ${rqt_cb_gui_UIS_H})
target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

