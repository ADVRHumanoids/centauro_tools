cmake_minimum_required(VERSION 2.8.3)
project(centauro_tools)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs 
                                        robot_state_publisher 
                                        tf_conversions 
                                        roscpp 
                                        std_msgs 
                                        sensor_msgs 
                                        std_srvs 
                                        geometry_msgs 
                                        message_generation 
                                        eigen_conversions
                                        cartesian_interface
                                        )

find_package(OpenSoT REQUIRED)
find_package(XCM REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)


# # add_action_files(
# #   DIRECTORY action
# #   FILES LegMovement.action
# # )
# # 
# # generate_messages(
# #   DEPENDENCIES
# #   std_msgs actionlib_msgs
# # )

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


add_message_files(
DIRECTORY msgs
FILES
HeriHandControl.msg
)

add_message_files(
DIRECTORY msgs
FILES
HeriHandState.msg
)

add_message_files(
DIRECTORY msgs
FILES
HeriHandFinger.msg
)

add_message_files(
DIRECTORY msgs
FILES
HeriHandFingerAnalogs.msg
)
    
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
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS robot_state_publisher 
                roscpp 
                actionlib_msgs
   DEPENDS
)

###########
## Build ##
###########

# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${OpenSoT_INCLUDE_DIRS} 
    ${cartesian_interface_INCLUDE_DIRS})

add_executable(simple_wheeled_motion src/simple_wheeled_motion.cpp)
target_link_libraries(simple_wheeled_motion ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable(unicycle_tracking src/UnicycleTracking.cpp)
target_link_libraries(unicycle_tracking ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable(centauro_polygon_publisher src/PolygonPublisher.cpp)
target_link_libraries(centauro_polygon_publisher ${catkin_LIBRARIES})

add_executable(centauro_cop_estimation src/CopEstimation.cpp)
target_link_libraries(centauro_cop_estimation ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(centauro_force_estimation src/ForceEstimation.cpp)
target_link_libraries(centauro_force_estimation ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(imu_orientation_control src/ImuOrientationControl.cpp)
target_link_libraries(imu_orientation_control ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(centauro_suspensions src/CentauroSuspensionModule.cpp)
target_link_libraries(centauro_suspensions ${catkin_LIBRARIES} ${OpenSoT_LIBRARIES} ${XBotInterface_LIBRARIES})

add_executable(centauro_reset_world src/CentauroCiResetWorld.cpp)
target_link_libraries(centauro_reset_world ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})
add_executable(velocity_motion src/velocity_motion.cpp)
target_link_libraries(velocity_motion ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_library(CartesianWheeledMotion src/SimpleWheeledMotion.cpp)
target_link_libraries(CartesianWheeledMotion ${cartesian_interface_LIBRARIES} ${OpenSoT_LIBRARIES} yaml-cpp)

add_library(HeriHand SHARED src/HeriHand.cpp)
target_link_libraries(HeriHand ${XCM_LIBRARIES})
ycm_library_install(HeriHand 1 0 0)


add_library(CentauroCartesioAddon SHARED src/CartesioAddon.cpp)
target_link_libraries(CentauroCartesioAddon ${cartesian_interface_LIBRARIES} ${OpenSoT_LIBRARIES} yaml-cpp)
ycm_library_install(CentauroCartesioAddon 1 0 0)

#############
## Install ##
#############


if ( Xenomai_FOUND )
    set_xeno_flags(HeriHand)
endif()

install(TARGETS unicycle_tracking unicycle_tracking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS centauro_reset_world centauro_reset_world
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS centauro_polygon_publisher centauro_polygon_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS velocity_motion velocity_motion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS centauro_suspensions centauro_suspensions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS centauro_cop_estimation centauro_cop_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS centauro_force_estimation centauro_force_estimation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS CartesianWheeledMotion CartesianWheeledMotion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS imu_orientation_control imu_orientation_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/joy_ci.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_centauro_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
