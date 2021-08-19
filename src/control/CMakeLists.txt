cmake_minimum_required(VERSION 2.8.12)
project(control)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  roscpp
  tf
  volksbot
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
)

add_service_files(
  FILES
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime roscpp 
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  launch
)

add_executable(kb
  src/keyboard/kb_control.cc
  src/keyboard/KbControl.cpp
)
set_target_properties(kb PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(kb
  ${catkin_LIBRARIES}
)

add_executable(js
  src/joystick/jscontrol.cc
  src/joystick/joystick.cc
  src/joystick/firejet.cc
  src/joystick/predator.cc
  src/joystick/logitechf710.cc
)
set_target_properties(js PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(js
  ${catkin_LIBRARIES}
)

add_executable(gio 
  src/gio_path/curves.cpp
  src/gio_path/gio_path.cpp
  src/gio_path/InputGio.cpp
  src/gio_path/input_gio.cc
)
set_target_properties(gio PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(gio
  ${catkin_LIBRARIES}
)

MESSAGE(STATUS "STATUS: ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: include/${PROJECT_NAME}")
MESSAGE(STATUS "STATUS: ${CATKIN_DEVEL_PREFIX}/include/")
