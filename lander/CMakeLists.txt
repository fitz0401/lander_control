cmake_minimum_required(VERSION 3.12)
project(lander)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_COMPILER "/usr/bin/gcc-8")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-8")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find aris
add_definitions(-DUNIX)
find_package(aris REQUIRED PATHS /usr/aris)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# 需要加入 message_generation,必须有 std_msgs

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
add_message_files(
  FILES
  gait_plan_msgs.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  gait_feedback_msgs.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

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
#  LIBRARIES movemessage
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${aris_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/movemessage.cpp
# )
add_executable(lander_cmd  src/lander_cmd.cpp src/ControlCMD.cpp include/ControlCMD.h include/general.hpp include/CubicSplineNoDynamic.cpp include/CubicSplineNoDynamic.h include/GetPosIK.cpp include/GetPosIK.h include/Param.h include/TransMechPositionIK.h include/TransMechPositionIK.cpp include/WalkLegExeLegIk.cpp include/WalkLegExeLegIk.h)
add_executable(lander_plan  src/lander_plan.cpp src/ControlPlan.cpp include/ControlPlan.h include/general.hpp include/CubicSplineNoDynamic.cpp include/CubicSplineNoDynamic.h include/GetPosIK.cpp include/GetPosIK.h include/Param.h include/TransMechPositionIK.h include/TransMechPositionIK.cpp include/WalkLegExeLegIk.cpp include/WalkLegExeLegIk.h)
add_executable(lander_RTplan  src/lander_RTplan.cpp src/ControlRTPlan.cpp include/ControlRTPlan.h include/general.hpp include/CubicSplineNoDynamic.cpp include/CubicSplineNoDynamic.h include/GetPosIK.cpp include/GetPosIK.h include/Param.h include/TransMechPositionIK.h include/TransMechPositionIK.cpp include/WalkLegExeLegIk.cpp include/WalkLegExeLegIk.h)
add_executable(mytest src/test.cpp)
add_executable(Test_joint_pub src/Test_joint_pub.cpp)		
add_executable(test_mul_threads src/test_mul_threads.cpp)		


# add_executable(mytestserver src/mytestserver.cpp)
# add_executable(mytestclient src/mytestclient.cpp)
# add_executable(mytest_srv_client src/mytest_srv_client.cpp)
# add_executable(final_control_server src/final_control_server.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(mytestclient ${PROJECT_NAME}_gencpp)
# add_dependencies(mytest_srv_client ${PROJECT_NAME}_gencpp)
# add_dependencies(final_control_server ${PROJECT_NAME}_gencpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/movemessage_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(mytestclient  ${catkin_LIBRARIES})
# target_link_libraries(mytest_srv_client  ${catkin_LIBRARIES})
# target_link_libraries(final_control_server  ${catkin_LIBRARIES})


# target_include_directories(movemessage PUBLIC  ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(lander_cmd  ${aris_LIBRARIES}  )
target_link_libraries(lander_cmd  ${catkin_LIBRARIES})
target_include_directories(lander_cmd PUBLIC ${kaanh_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lander_cmd ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic pthread stdc++fs)
target_link_libraries(lander_cmd ${aris_LIBRARIES} atomic modbus /usr/local/lib/libmodbus.so.5)

target_link_libraries(lander_plan  ${aris_LIBRARIES}  )
target_link_libraries(lander_plan  ${catkin_LIBRARIES})
target_include_directories(lander_plan PUBLIC ${kaanh_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lander_plan ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic pthread stdc++fs)
target_link_libraries(lander_plan ${aris_LIBRARIES} atomic modbus /usr/local/lib/libmodbus.so.5)

target_link_libraries(lander_RTplan  ${aris_LIBRARIES}  )
target_link_libraries(lander_RTplan  ${catkin_LIBRARIES})
target_include_directories(lander_RTplan PUBLIC ${kaanh_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lander_RTplan ${kaanh_LIBRARIES} ${aris_LIBRARIES} atomic pthread stdc++fs)
target_link_libraries(lander_RTplan ${aris_LIBRARIES} atomic modbus /usr/local/lib/libmodbus.so.5)


target_link_libraries(mytest  ${catkin_LIBRARIES})
target_link_libraries(Test_joint_pub ${catkin_LIBRARIES})
target_link_libraries(test_mul_threads ${catkin_LIBRARIES})
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_movemessage.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
