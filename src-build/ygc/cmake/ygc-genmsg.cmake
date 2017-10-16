# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ygc: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iygc:/home/ros16/WorkSpace/bearing_ws/src/ygc/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ygc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" NAME_WE)
add_custom_target(_ygc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ygc" "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" "ygc/Bearing2D"
)

get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" NAME_WE)
add_custom_target(_ygc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ygc" "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg"
  "${MSG_I_FLAGS}"
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ygc
)
_generate_msg_cpp(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ygc
)

### Generating Services

### Generating Module File
_generate_module_cpp(ygc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ygc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ygc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ygc_generate_messages ygc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" NAME_WE)
add_dependencies(ygc_generate_messages_cpp _ygc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" NAME_WE)
add_dependencies(ygc_generate_messages_cpp _ygc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ygc_gencpp)
add_dependencies(ygc_gencpp ygc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ygc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg"
  "${MSG_I_FLAGS}"
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ygc
)
_generate_msg_eus(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ygc
)

### Generating Services

### Generating Module File
_generate_module_eus(ygc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ygc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ygc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ygc_generate_messages ygc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" NAME_WE)
add_dependencies(ygc_generate_messages_eus _ygc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" NAME_WE)
add_dependencies(ygc_generate_messages_eus _ygc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ygc_geneus)
add_dependencies(ygc_geneus ygc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ygc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg"
  "${MSG_I_FLAGS}"
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ygc
)
_generate_msg_lisp(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ygc
)

### Generating Services

### Generating Module File
_generate_module_lisp(ygc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ygc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ygc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ygc_generate_messages ygc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" NAME_WE)
add_dependencies(ygc_generate_messages_lisp _ygc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" NAME_WE)
add_dependencies(ygc_generate_messages_lisp _ygc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ygc_genlisp)
add_dependencies(ygc_genlisp ygc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ygc_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg"
  "${MSG_I_FLAGS}"
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc
)
_generate_msg_py(ygc
  "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc
)

### Generating Services

### Generating Module File
_generate_module_py(ygc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ygc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ygc_generate_messages ygc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/GroupBearing.msg" NAME_WE)
add_dependencies(ygc_generate_messages_py _ygc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ros16/WorkSpace/bearing_ws/src/ygc/msg/Bearing2D.msg" NAME_WE)
add_dependencies(ygc_generate_messages_py _ygc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ygc_genpy)
add_dependencies(ygc_genpy ygc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ygc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ygc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ygc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ygc_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ygc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ygc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(ygc_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ygc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ygc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ygc_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc)
  install(CODE "execute_process(COMMAND \"/usr/bin/X11/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ygc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ygc_generate_messages_py std_msgs_generate_messages_py)
