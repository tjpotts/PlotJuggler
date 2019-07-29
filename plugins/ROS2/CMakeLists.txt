
include_directories( ../ )

# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#############
add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)
#############

SET( COMMON_SRC
    dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    dialog_with_itemlist.h
    )

QT5_WRAP_UI ( COMMON_UI_SRC
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    )

add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC} )
target_link_libraries( commonROS
  ${Qt5Widgets_LIBRARIES}
  ${Qt5Xml_LIBRARIES}
  )
ament_target_dependencies( commonROS rclcpp rosbag2)


#############
SET( DATALOAD_SRC
    DataLoadROS/dataload_ros.cpp
    ../../include/PlotJuggler/dataloader_base.h
    )

add_library( DataLoadROS SHARED ${DATALOAD_SRC}  )
target_link_libraries( DataLoadROS  commonROS)

install(TARGETS DataLoadROS
    DESTINATION lib/plotjuggler  )
