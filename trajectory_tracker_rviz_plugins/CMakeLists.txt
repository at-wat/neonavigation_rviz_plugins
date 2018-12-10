cmake_minimum_required(VERSION 2.8.3)
project(trajectory_tracker_rviz_plugins)

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    rviz
    trajectory_tracker_msgs
)
catkin_package(
  CATKIN_DEPENDS
    pluginlib
    trajectory_tracker_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})

  qt4_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

  qt5_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/path_with_velocity_display.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)