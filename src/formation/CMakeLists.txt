project(formation)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE "Release") #using CMAKE_BUILD_TYPE to set the cmake method [Debug | Release]
# CMAKE_CXX_FLAGS CMake C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -pthread -fPIE")  #-fopenmp add-on multi-thread openmp support
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  )


set(CMAKE_INCLUDE_CURRENT_DIR ON)
aux_source_directory(. SRC_LIST)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    dynamic_reconfigure
    message_generation
    roscpp
    rospy
    std_msgs
    std_srvs
    mavros
    mavros_msgs
    sensor_msgs
    ygc
    )

set(formation_includes
    include
    ${catkin_INCLUDE_DIRS}
    )

include_directories(${formation_includes}    )

catkin_package(
    INCLUDE_DIRS   ${formation_includes}
    CATKIN_DEPENDS
 dynamic_reconfigure  message_generation  roscpp rospy  std_msgs std_srvs mavros ygc
    )

add_library(formation  STATIC
src/formation.cpp
include/formation.h
)
target_link_libraries(formation  ${catkin_LIBRARIES} ygc

)
add_dependencies(formation  ${catkin_EXPORTED_TARGETS} ygc)
add_executable(formation_node
               include/formation.h
               src/main.cpp
               src/formation.cpp
    )

target_link_libraries(formation_node
    ${catkin_LIBRARIES}
   ${CMAKE_THREAD_LIBS_INIT}
    formation
    ygc
)
add_dependencies(formation_node ${catkin_EXPORTED_TARGETS} ygc)


###############INSTALL#####################
install(TARGETS formation formation_node
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
