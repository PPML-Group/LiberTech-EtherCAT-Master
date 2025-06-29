# Copyright 2021 Robotic Systems Lab - ETH Zurich:
# Lennart Nachtigall, Jonas Junger
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


cmake_minimum_required(VERSION 3.16)
project(ethercat_device_configurator)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 17)
add_definitions(-Wall -Werror -Wextra)
add_definitions(-DMELO_USE_COUT)

set(PACKAGE_DEPENDENCIES
)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED
    COMPONENTS
    param_io
    ethercat_sdk_master)

find_package(ethercat_sdk_master REQUIRED)
find_package(param_io REQUIRED)

# find_package(anydrive_rsl QUIET)
find_package(elmo_ethercat_sdk QUIET)
# find_package(mps_ethercat_sdk QUIET)
# find_package(maxon_epos_ethercat_sdk QUIET)
# find_package(rokubimini_rsl_ethercat_slave QUIET)
# find_package(ek1100 QUIET)
# find_package(el3102 QUIET)

if (anydrive_rsl_FOUND)
  add_definitions(-D_ANYDRIVE_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES anydrive_rsl)
endif ()
if (rokubimini_rsl_ethercat_slave_FOUND)
  add_definitions(-D_ROKUBI_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES rokubimini_rsl_ethercat_slave)
endif ()
if (elmo_ethercat_sdk_FOUND)
  add_definitions(-D_ELMO_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES elmo_ethercat_sdk)
endif ()
if (mps_ethercat_sdk_FOUND)
  add_definitions(-D_MPSDRIVE_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES mps_ethercat_sdk)
endif ()
if (maxon_epos_ethercat_sdk_FOUND)
  add_definitions(-D_MAXON_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES maxon_epos_ethercat_sdk)
endif ()
if (ek1100_FOUND)
  add_definitions(-D_EK1100_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES ek1100)
endif ()
if (el3102_FOUND)
  add_definitions(-D_EL3102_FOUND_)
  LIST(APPEND PACKAGE_DEPENDENCIES el3102)
endif ()

foreach (sdk ${PACKAGE_DEPENDENCIES})
  message("Found EtherCAT device sdk: " ${sdk})
endforeach ()

LIST(APPEND PACKAGE_DEPENDENCIES
    param_io
    ethercat_sdk_master)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${anydrive_rsl_INCLUDE_DIRS}
    ${elmo_ethercat_sdk_INCLUDE_DIRS}
    ${mps_ethercat_sdk_INCLUDE_DIRS}
    ${maxon_epos_ethercat_sdk_INCLUDE_DIRS}
    ${rokubimini_rsl_ethercat_slave_INCLUDE_DIRS}
    ${ek1100_INCLUDE_DIRS}
    ${el3102_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ./src/EthercatDeviceConfigurator.cpp
)


add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )


target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${anydrive_rsl_LIBRARIES}
    ${elmo_ethercat_sdk_LIBRARIES}
    ${mps_ethercat_sdk_LIBRARIES}
    ${maxon_epos_ethercat_sdk_LIBRARIES}
    ${rokubimini_rsl_ethercat_slave_LIBRARIES}
    ${ek1100_LIBRARIES}
    ${el3102_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    stdc++fs
)


add_executable(
  standalone
  src/standalone.cpp
)

add_dependencies(
    standalone
    ${PROJECT_NAME}
    ${anydrive_rsl_EXPORTED_TARGETS}
    ${rokubimini_rsl_ethercat_slave_EXPORTED_TARGETS}
    ${elmo_ethercat_sdk_EXPORTED_TARGETS}
    ${mps_ethercat_sdk_EXPORTED_TARGETS}
    ${maxon_epos_ethercat_sdk_EXPORTED_TARGETS}
    ${ek1100_EXPORTED_TARGETS}
    ${el3102_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    standalone
    ${PROJECT_NAME}
    ${anydrive_rsl_LIBRARIES}
    ${elmo_ethercat_sdk_LIBRARIES}
    ${mps_ethercat_sdk_LIBRARIES}
    ${maxon_epos_ethercat_sdk_LIBRARIES}
    ${rokubimini_rsl_ethercat_slave_LIBRARIES}
    ${ek1100_LIBRARIES}
    ${el3102_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    -pthread
    stdc++fs
)

install(TARGETS ${PROJECT_NAME} #standalone
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )



