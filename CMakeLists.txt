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
                                        )

# # find_package(OpenSoT REQUIRED)

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



# # catkin_package(
# # #  INCLUDE_DIRS include
# #    LIBRARIES 
# #    CATKIN_DEPENDS robot_state_publisher 
# #                 roscpp 
# #                 actionlib_msgs
# #    DEPENDS
# # )

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


# # include_directories(include ${catkin_INCLUDE_DIRS} ${OpenSoT_INCLUDE_DIRS})
# # 
# # add_executable(wholebody_action_server src/wholebody_action_server.cpp)
# # add_dependencies(wholebody_action_server roscpp_generate_messages_cpp)
# # target_link_libraries(wholebody_action_server ${Boost_LIBRARIES} 
# #                                               ${catkin_LIBRARIES} 
# #                                               ${OpenSoT_LIBRARIES} 
# #                                               ${XBotInterface_LIBRARIES})
# # 
# # add_executable(simple_wheeled_motion src/simple_wheeled_motion.cpp)
# # target_link_libraries(simple_wheeled_motion ${XBotInterface_LIBRARIES})

include_directories(include)

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

add_library(HeriHand SHARED src/HeriHand.cpp)
target_link_libraries(HeriHand ${XCM_LIBRARIES})
ycm_library_install(HeriHand 1 0 0)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# # install(TARGETS wholebody_action_server wholebody_action_server
# #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )
# # 
# # install(TARGETS simple_wheeled_motion simple_wheeled_motion
# #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )
# # 
# # # Mark other files for installation (e.g. launch and bag files, etc.)
# # install(FILES
# #   launch/plot_centauro_effort.launch
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# # )

if ( Xenomai_FOUND )
    set_xeno_flags(HeriHand)
endif()

