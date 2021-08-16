cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
  message_generation
  roscpp
  tf
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  pose2d.msg
  ticks.msg
  vels.msg
  vel_limit.msg
)

add_service_files(
  FILES
  velocities.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
add_dependencies(kb control_generate_messages_cpp)

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
add_dependencies(js control_generate_messages_cpp)

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
add_dependencies(gio control_generate_messages_cpp)

add_executable(gio_tf
  src/gio_path/gio_start_tf.cc
  src/gio_path/GioFrame.cpp
)
set_target_properties(gio_tf PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(gio_tf
  ${catkin_LIBRARIES}
)
add_dependencies(gio_tf control_generate_messages_cpp)

MESSAGE(STATUS "STATUS: ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
MESSAGE(STATUS "STATUS: include/${PROJECT_NAME}")
MESSAGE(STATUS "STATUS: ${CATKIN_DEVEL_PREFIX}/include/")

#install(DIRECTORY include/${PROJECT_NAME}
#	DESTINATION ${CATKIN_DEVEL_PREFIX}/include/
#	PATTERN ".svn" EXCLUDE)

#DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/control
#)

# install(TARGETS odo VMCLIB DESTINATION lib EXPORT control-targets)
# install(EXPORT control-targets DESTINATION lib/control)
