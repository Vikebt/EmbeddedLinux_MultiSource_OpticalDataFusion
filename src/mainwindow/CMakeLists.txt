cmake_minimum_required(VERSION 3.0.2)
project(window_control)

## Embedded Linux default: optimized, debuggable symbols are opt-in via Debug.
add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  sbg_driver
  serial
  serial_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#寻找Qt的包
find_package(Qt5  REQUIRED COMPONENTS  Widgets Core Gui Svg)

find_package(yaml-cpp REQUIRED) 
find_package(OpenCV REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.8 REQUIRED )
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# 查找libudev库
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

#设置自动生成moc文件，一定要设置
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)#设置工程包含当前目录，非必须

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs#   tf2_geometry_msgs#   tf2_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sbg_subscribe
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs std_srvs tf2_geometry_msgs tf2_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/location/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Integrated_Navigation_sys/include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS} # 添加此行
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sbg_subscribe.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/sbg_subscribe_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sbg_subscribe.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#add_executable(gnss_fusion src/gnss_fusion.cpp)
#target_link_libraries(gnss_fusion
#  ${catkin_LIBRARIES}
#)
# 链接库
#target_link_libraries(gnss_fusion
#  ${catkin_LIBRARIES}
#  ${YAML_CPP_LIBRARIES} # 添加此行
#  ${OpenCV_LIBRARIES}
#)
#add_dependencies(gnss_fusion sbg_driver_generate_messages_cpp)

#头文件
set(HEADERS
	include/mainwindow.h
	include/gimbal_control.h
	include/gnss_output.h
	#include/px4_serial.h
	include/rtsp_capture.h
	include/rtsp_url_config.h
	include/yaml_config.h
	include/qnode.h
	include/thread_control.h
	include/radio_altitude.h
	include/air_pressure.h
	include/dlgSerialSetting.h
	include/thread_scene_match.h
	include/dlgImageMatchShow.h
	include/thread_terrain_match.h
	include/point_cloud_capture.h
	include/threadPointcloud.h
	src/location/include/matching.h
	src/location/include/TensorMatch.h
	src/Integrated_Navigation_sys/include/data_struct.h
    	src/Integrated_Navigation_sys/include/Height_fusion.h
    	src/Integrated_Navigation_sys/include/INS_update.h
    	src/Integrated_Navigation_sys/include/Mat_Math.h
    	src/Integrated_Navigation_sys/include/Kalman.h
    	src/Integrated_Navigation_sys/include/SITAN.h
    	src/Integrated_Navigation_sys/include/muti_thread.h
    	src/Integrated_Navigation_sys/include/TERCOM.h
	src/qfi/qfi_defs.h
	src/qfi/qfi_doxygen.h
	src/qfi/qfi_Colors.h
	src/qfi/qfi_Fonts.h
	src/qfi/qfi_EADI.h
	src/qfi/qfi_EHSI.h
	src/qfi/qfi_AI.h
	src/qfi/qfi_HI.h
	src/qfi/qfi_VSI.h 
	src/qfi/qfi_ASI.h
	src/qfi/qfi_ALT.h
	src/qfi/qfi_TC.h
	)
#源文件
set(SOURCES
        src/main.cpp
        src/mainwindow.cpp
        src/mainwindow.ui
        src/gimbal_control.cpp
        src/gnss_output.cpp
        #src/px4_serial.cpp
        src/rtsp_capture.cpp
        src/yaml_config.cpp
        src/qnode.cpp
        src/thread_control.cpp
        src/radio_altitude.cpp
	src/air_pressure.cpp
	src/dlgSerialSetting.cpp
	src/dlgSerialSetting.ui
	src/thread_scene_match.cpp
	src/thread_terrain_match.cpp
	src/dlgImageMatchShow.cpp
	src/dlgImageMatchShow.ui
	src/point_cloud_capture.cpp
	src/threadPointcloud.cpp
	src/location/src/matching.cpp
	src/location/src/TensorMatch.cpp
	src/Integrated_Navigation_sys/src/Height_fusion.cpp
    	src/Integrated_Navigation_sys/src/INS_update.cpp 
    	src/Integrated_Navigation_sys/src/Kalman.cpp
    	src/Integrated_Navigation_sys/src/Mat_Math.cpp 
    	src/Integrated_Navigation_sys/src/muti_thread.cpp 
    	src/Integrated_Navigation_sys/src/SITAN.cpp
   	src/Integrated_Navigation_sys/src/TERCOM.cpp
	src/qfi/qfi_Colors.cpp
	src/qfi/qfi_Fonts.cpp
	src/qfi/qfi_EADI.cpp
	src/qfi/qfi_EHSI.cpp
	src/qfi/qfi_AI.cpp
	src/qfi/qfi_HI.cpp
	src/qfi/qfi_VSI.cpp
	src/qfi/qfi_ASI.cpp
	src/qfi/qfi_ALT.cpp
	src/qfi/qfi_TC.cpp
        )

file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h)

#资源文件
set(QT_RESOURCES
        resources/image.qrc
        src/qfi/qfi.qrc
        )

#link_directories(/home/l/GuaFei/qt_serial_ws/src/mainwindow/src/location/libs)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
file(GLOB_RECURSE RESOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(window_control ${HEADERS} ${SOURCES} ${QT_RESOURCES_CPP} ${QT_RESOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})

#在定义消息类型时，编译的可执行文件依赖这些动态生成的代码，需要使用${PROJECT_NAME}_generate_messages_cpp进行配置。
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
#设置可执行文件依赖的库。
#target_link_libraries (window_control TensorMatch)
target_link_libraries(window_control ${catkin_LIBRARIES})
target_link_libraries(window_control  Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Svg)#链接Qt库
target_link_libraries (window_control ${OpenCV_LIBS})
target_link_libraries (window_control ${YAML_CPP_LIBRARIES})
target_link_libraries (window_control ${PCL_LIBRARIES})
target_link_libraries (window_control ${UDEV_LIBRARIES})
target_include_directories(window_control PRIVATE ${UDEV_INCLUDE_DIRS})
add_dependencies(window_control sbg_driver_generate_messages_cpp)

install(TARGETS window_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
