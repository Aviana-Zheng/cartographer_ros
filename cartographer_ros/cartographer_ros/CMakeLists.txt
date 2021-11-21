# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

project(cartographer_ros)

set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
set(BUILD_SHARED_LIBS OFF)
google_initialize_cartographer_project()
google_enable_testing()

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# pkg-config 就是一个命令工具，可以查询已安装库的基本信息，包括版本号、库路径、头文件路径、编译选项、链接选项等， 是一个在源代码编译时查询已安装的库的使用接口的计算机工具软件。
# pkg-config --modversion glew  # 查询glew版本号
# pkg-config --libs --cflags glew # 获取glew库的基本信息
# find_package(PkgConfig)
# pkg_search_module(GLEW REQUIRED glew)
# MESSAGE(STATUS "glew dirs:" ${GLEW_INCLUDE_DIRS})
# MESSAGE(STATUS "glew lib:" ${GLEW_LIBRARIES})
# include_directories(${GLEW_INCLUDE_DIRS})
# link_directories(${GLEW_LIBRARY_DIRS})
# ... ...
# target_link_libraries(main ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${GLEW_LIBRARIES})

# pkg-config方式
# GLEW_INCLUDE_DIRS 
# GLEW_LIBRARY_DIRS
# GLEW_LIBRARIES

# find_package
# GLEW_INCLUDE_DIR
# GLEW_LIBRARY_DIR
# GLEW_LIBRARY

include(FindPkgConfig)
pkg_search_module(YAMLCPP REQUIRED yaml-cpp>=0.5.1)

find_package(LuaGoogle REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # TODO(damonkohler): This should be here but causes Catkin to abort because
    # protobuf specifies a library '-lpthread' instead of just 'pthread'.
    # CARTOGRAPHER
    YAMLCPP
    PCL
    EIGEN3
    Boost
    urdfdom_headers
  INCLUDE_DIRS "."
  LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
add_library(${PROJECT_NAME} ${ALL_SRCS})
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)

# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
foreach(DEFINITION ${PCL_DEFINITIONS})
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# Catkin
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

if (CATKIN_ENABLE_TESTING)
  foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
    #get_filename_component(<var>  <Filename>  <mode>)
    #<var>：保存获取部分的变量
    #<Filename>：完整的文件名
    # NAME_WE：没有目录和扩展名的文件名
    get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
    catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
    # catkin_add_gtest uses a plain (i.e. no PUBLIC/PRIVATE/INTERFACE) call to
    # target_link_libraries. That forces us to do the same.
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
    target_link_libraries(${TEST_NAME} ${LUA_LIBRARIES})
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
    # add_dependencies()：添加依赖项，在使用ROS的message、service、action时注意添加：

    # 添加对其它package消息的依赖
    # 前提是已经通过find_package()引入了这个package 
    # add_dependencies(my_target ${catkin_EXPORTED_TARGETS}) 
    
    # 添加对本package消息的依赖 
    # add_dependencies(my_target ${${PROJECT_NAME}_EXPORTED_TARGETS})
    
    # 为顶层目标引入一个依赖关系。让一个顶层目标依赖于其他的顶层目标。
    # 一个顶层目标是由命令ADD_EXECUTABLE，ADD_LIBRARY，或者ADD_CUSTOM_TARGET产生的目标。
    # 为这些命令的输出引入依赖性可以保证某个目标在其他的目标之前被构建。
    add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${TEST_NAME} cartographer)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
    set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
  endforeach()
endif()

# 和CMakeLists的同级目录下的launch urdf configuration_files文件夹中的文件安装到
# catkin工作空间下的install或者install_isolated下share/${PROJECT_NAME}下
install(DIRECTORY launch urdf configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# PROGRAMS指的是非目标文件的可执行程序(如脚本文件)。
install(PROGRAMS scripts/tf_remove_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install source headers.
file(GLOB_RECURSE HDRS "*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()
