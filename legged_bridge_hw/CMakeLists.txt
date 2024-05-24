cmake_minimum_required(VERSION 3.10)
project(legged_bridge_hw)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-Wall -Werror)
add_definitions(-DMELO_USE_COUT)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        rospy
        dynamic_reconfigure
        geometry_msgs
        sensor_msgs
        std_msgs
        std_srvs
        message_generation
        tf)


find_package(elmo_ethercat_sdk )

if(elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  set(PACKAGE_DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
    elmo_ethercat_sdk
  )
endif()


catkin_package(
        INCLUDE_DIRS
        include
        app
        CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
        roscpp
        legged_hw)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${anydrive_INCLUDE_DIRS}
        ${elmo_ethercat_sdk_INCLUDE_DIRS}
        ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
        ${rokubimini_rsl_ethercat_INCLUDE_DIRS}
        app
        app/ethercat
        app/ethercat/config
        app/ethercat/config/device_configurations
        # app/thirdparty/soem/soem
        # app/thirdparty/soem/osal
        # app/thirdparty/soem/osal/linux
        # app/thirdparty/soem/oshw/linux
        # app/thirdparty/cpp-readline/src
)

## Declare cpp executables
add_executable(legged_bridge_hw 
        src/legged_bridge_hw.cpp
        src/BridgeHW.cpp
        app/ethercat/EthercatDeviceConfigurator.cpp)

## Specify libraries to link executable targets against
target_link_libraries(legged_bridge_hw
        ${catkin_LIBRARIES}
        ${elmo_ethercat_sdk_LIBRARIES} 
        ${YAML_CPP_LIBRARIES}
        stdc++fs
        -pthread) 


find_package(Boost REQUIRED)

# set(CMAKE_CXX_STANDARD 17)

# set(SOURCES app/motor_control.c
#             app/motor_control.h
#             app/math_ops.c
#             app/math_ops.h
#             app/transmit.cpp
#             app/transmit.h
#             app/config.h
#             app/queue.h 
#             app/command.cpp 
#             app/command.h)

# add_library(motor ${SOURCES})
# target_link_libraries(motor soem)

# set(SOURCES app/ethercat/EthercatDeviceConfigurator.cpp
#             app/ethercat/EthercatDeviceConfigurator.hpp)

# add_library(ethercat ${SOURCES})

# # add_executable(master_stack_test main.cpp)
# target_link_libraries(legged_bridge_hw 
#                         ethercat
#                         # cpp-readline   
#                         ${elmo_ethercat_sdk_LIBRARIES} 
#                         ${YAML_CPP_LIBRARIES}
#                         stdc++fs
#                         -pthread)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " legged_bridge_hw)
    add_clang_tooling(
            TARGETS legged_unitree_hw_go1
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation

install(TARGETS legged_bridge_hw
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )



