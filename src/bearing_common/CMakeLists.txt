cmake_minimum_required(VERSION 2.8.3)
project(bearing_common)

set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
)



add_message_files(
FILES
Bearing2D.msg
GroupBearing.msg
Vector2D.msg
DataRecord.msg
AllPosition.msg
TriggerRec.msg
)


add_service_files(DIRECTORY srvs/commander
  FILES
  HParamSetSrv.srv

)

generate_messages(
DEPENDENCIES
std_msgs geometry_msgs )


set(bearing_common_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/core/include 
)
include_directories(${bearing_common_includes})
catkin_package(
  INCLUDE_DIRS ${smarteye_common_includes}
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/core/include
  LIBRARIES bearing_common
  CATKIN_DEPENDS   message_filters  message_generation  roscpp  rospy  std_msgs  std_srvs  geometry_msgs mavros_msgs
  message_runtime
)
add_library(bearing_common  STATIC
                             core/src/MathCal.cpp

)
add_dependencies(bearing_common  ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_common
    ${catkin_LIBRARIES}
)



#########install





