cmake_minimum_required(VERSION 3.2)
project(ygc)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  )




find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_generation
    roscpp
    rospy
    std_msgs
    std_srvs
    mavros
    mavros_msgs
    sensor_msgs
bearing_common
    )




catkin_package(
    INCLUDE_DIRS  include
    LIBRARIES ygc
    CATKIN_DEPENDS   roscpp rospy  std_msgs std_srvs mavros
message_runtime bearing_common)

#set(ygc_includes
#    include
#    ${catkin_INCLUDE_DIRS})


include_directories(include ${catkin_INCLUDE_DIRS})


add_library(ygc  STATIC
src/ygc.cpp
include/ygc.h
)
target_link_libraries(ygc  ${catkin_LIBRARIES})
add_dependencies(ygc  ${catkin_EXPORTED_TARGETS} )
add_executable(ygc_node
               src/main.cpp
               src/ygc.cpp
    )

target_link_libraries(ygc_node
    ${catkin_LIBRARIES}
)
add_dependencies(ygc_node ${catkin_EXPORTED_TARGETS} )


###############INSTALL#####################
install(TARGETS   ygc ygc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
