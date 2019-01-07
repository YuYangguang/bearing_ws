cmake_minimum_required(VERSION 2.8.3)
project(bearing_common)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  )


set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(. SRC_LIST)

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
               ${PROJECT_SOURCE_DIR}/include
)
include_directories(${bearing_common_includes})
catkin_package(
  INCLUDE_DIRS ${bearing_common_includes}
  ${PROJECT_SOURCE_DIR}/include
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/core/include
  LIBRARIES bearing_common
  CATKIN_DEPENDS   message_filters  message_generation  roscpp  rospy  std_msgs  std_srvs  geometry_msgs mavros_msgs
  message_runtime
)
add_library(bearing_common  SHARED
                             src/3rd_party_header.cpp
                             core/src/MathCal.cpp
                             include/3rd_party_header.h


)
add_dependencies(bearing_common  ${catkin_EXPORTED_TARGETS})
target_link_libraries(bearing_common
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)



#########install
install(TARGETS bearing_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY core/include/smarteye/core/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/smarteye/core/
  FILES_MATCHING PATTERN "*.h*"
)




