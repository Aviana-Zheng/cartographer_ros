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

project(cartographer)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
# 使能gtest
google_enable_testing()

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Protobuf REQUIRED)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

# Only build the documentation if we can find Sphinx.
find_package(Sphinx)
if(SPHINX_FOUND)
  add_subdirectory("docs")
endif()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

# 和CMakeLists的同级目录下的configuration_files文件夹中的文件安装到share/cartographer下
# 如果程序报错，可以修改如下
# install(DIRECTORY configuration_files DESTINATION share/cartographer)  
# 错误 多了一个/  /usr/local/share/cartographer//configuration_files/trajectory_builder_3d.lua
install(DIRECTORY configuration_files DESTINATION share/cartographer/)

# 和CMakeLists的同级目录下的cmake文件夹中的文件安装到share/cartographer下
# 如果程序报错，可以修改如下
# 同上修改 install(DIRECTORY cmake DESTINATION share/cartographer)
# /usr/local/share/cartographer//cmake/modules/FindGMock.cmake
install(DIRECTORY cmake DESTINATION share/cartographer/)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

add_library(${PROJECT_NAME} ${ALL_SRCS})

configure_file(
  ${PROJECT_SOURCE_DIR}/cartographer/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/cartographer/common/config.h)

google_binary(cartographer_autogenerate_ground_truth
  SRCS
    cartographer/ground_truth/autogenerate_ground_truth_main.cc
)

google_binary(cartographer_compute_relations_metrics
  SRCS
    cartographer/ground_truth/compute_relations_metrics_main.cc
)

foreach(ABS_FIL ${ALL_TESTS})
  #RELATIVE_PAT推断出指定文件相对于特定目录的路径。file(RELATIVE_PATH variable directory file) 
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  #get_filename_component(<var>  <Filename>  <mode>)
  #<var>：保存获取部分的变量
  #<Filename>：完整的文件名
  #<mode>：要获取文件名的哪个部分。包括：
  # DIRECTORY：文件所在目录
  # NAME：没有目录的文件名字
  # EXT：最长的扩展名，即第一个.开始
  # NAME_WE：没有目录和扩展名的文件名
  # LAST_EXT：最后一个扩展名
  # NAME_WLE：没有目录和最后一个扩展名的文件名
  # PATH：同DIRECTORY，已过时，用于<=2.8.11版本
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  # Replace slashes as required for CMP0037.
  # string(REPLACE <match-string> <replace-string> <out-var> <input>...)
  string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
  # functions.cmake   定义了google_test函数其实就是add_test
  google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
endforeach()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${LUA_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# We expect find_package(Ceres) to have located these for us.
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${CAIRO_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} pthread)

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# TODO(damonkohler): Create a testing library.
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  "${GMOCK_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${GMOCK_LIBRARY})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# CMAKE_INSTALL_PREFIX，用于指定cmake install时的相对地址前缀
# cmake -DCMAKE_INSTALL_PREFIX=/usr ..
# set(CMAKE_INSTALL_PREFIX [your install path])

# install(TARGETS targets... 
# 参数中的TARGET可以是很多种目标文件，
# 最常见的是通过ADD_EXECUTABLE或者ADD_LIBRARY定义的目标文件，即可执行二进制、动态库、静态库：

# [EXPORT <export-name>]

# [[ARCHIVE|LIBRARY|RUNTIME|OBJECTS|FRAMEWORK|BUNDLE|
# PRIVATE_HEADER|PUBLIC_HEADER|RESOURCE]
# 目标文件	         内容	                安装目录变量	                 默认安装文件夹
# ARCHIVE	         静态库	              ${CMAKE_INSTALL_LIBDIR}       	lib
# LIBRARY	         动态库	              ${CMAKE_INSTALL_LIBDIR}	        lib
# RUNTIME	        可执行二进制文件	      ${CMAKE_INSTALL_BINDIR}	        bin
# PUBLIC_HEADER	  与库关联的PUBLIC头文件	${CMAKE_INSTALL_INCLUDEDIR}	    include
# PRIVATE_HEADER	与库关联的PRIVATE头文件	${CMAKE_INSTALL_INCLUDEDIR}	    include
# 例子：
# INSTALL(TARGETS  mystaticlib mylib myrun
#        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )
# 静态库libmystaticlib.a安装到${CMAKE_INSTALL_LIBDIR}目录，
# 动态库libmylib.so安装到${CMAKE_INSTALL_LIBDIR}目录，
# 可执行二进制myrun安装到${CMAKE_INSTALL_BINDIR}目录。

# [CONFIGURATIONS [Debug|Release|...]]
# 指定安装规则适用的构建配置列表(DEBUG或RELEASE等)
# DEBUG和RELEASE版本的DESTINATION安装路径不同，那么DESTINATION必须在CONFIGUATIONS后面。

# [DESTINATION <dir>]
# 指定磁盘上要安装文件的目录；

# [PERMISSIONS permissions...]
# 指定安装文件的权限。有效权限是OWNER_READ，OWNER_WRITE，OWNER_EXECUTE，GROUP_READ，GROUP_WRITE，
# GROUP_EXECUTE，WORLD_READ，WORLD_WRITE，WORLD_EXECUTE，SETUID和SETGID；

# [COMPONENT <component>]
# [NAMELINK_COMPONENT <component>]
# [OPTIONAL] 
# 如果要安装的文件不存在，则指定不是错误。

# [EXCLUDE_FROM_ALL]
# 指定该文件从完整安装中排除，仅作为特定于组件的安装的一部分进行安装；

# [NAMELINK_ONLY|NAMELINK_SKIP]
# ] [...]
# [INCLUDES DESTINATION [<dir> ...]]
# )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT CartographerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install source headers.
file(GLOB_RECURSE hdrs "*.h")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${PROJECT_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

set(CARTOGRAPHER_CMAKE_DIR share/cartographer/cmake)
# CMakePackageConfigHelpers 用cmake建立自己的package
# CONFIGURE_PACKAGE_CONFIG_FILE(<input> <output> INSTALL_DESTINATION <path>
# [PATH_VARS <var1> <var2> ... <varN>]
# [NO_SET_AND_CHECK_MACRO]
# [NO_CHECK_REQUIRED_COMPONENTS_MACRO])

# input 输入文件  后缀为-config.cmake.in 或 Config.cmake.in

# output 输出文件 -config.cmake 或 Config.cmake

# INSTALL_DESTINATION    .cmake 文件的安装目录

# PATH_VARS  后面接的每一个变量都有形参、实参两种意义：
# 形参是指在cmake.in 中可以用 @PACKAGE_IVCOMMON_CMAKE_DIR@ @PACKAGE_INCLUDE_INSTALL_DIR@ 
# @PACKAGE_SYSCONFIG_INSTALL_DIR@来引用这些变量值
# 实参是指这些变量会将他们的值传递到函数内部，给形参。如果是相对路径也会自动转换为安装到的全局路径

include(CMakePackageConfigHelpers)
configure_package_config_file(
  cartographer-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  PATH_VARS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

# install(EXPORT <export-name> 
# 		DESTINATION <dir>
#         [NAMESPACE <namespace>] 
#         [FILE <name>.cmake]
#         [PERMISSIONS permissions...]
#         [CONFIGURATIONS [Debug|Release|...]]
#         [EXPORT_LINK_INTERFACE_LIBRARIES]
#         [COMPONENT <component>])


# EXPORT指定输出的配置target名，即在第一小节中用install(TARGETS)指定的<export-name>

# DESTINATION配置文件放到哪个目录，建议自己指定，之后好找

# NAMESPACE指定前缀，方便识别，使用时全名是<namespace><export-name>，如Qt5::Widgets的namespace是Qt5::

# FILE：文件名，必须以*.cmake结尾，如果在这些指定了路径，那么DESTINATION可空着

# PERMISSIONS：文件访问权限

# CONFIGURATIONS：默认为空，即全部生效，如指定特定配置，那么只在特定配置生效，
# 如指定release，则只在release时输出配置文件

# EXPORT_LINK_INTERFACE_LIBRARIES:如果使用，那么在CMP0022是NEW时，
# 只有匹配(IMPORTED_)?LINK_INTERFACE_LIBRARIES(_<CONFIG>)?的属性才会被输出

# COMPONENT：（没看明白，附上原文）If a COMPONENT option is specified that does not match 
# that given to the targets associated with the behavior is undefined. 
# If a library target is included in the export but a target to which it links is not 
# included the behavior is unspecified

install(
  EXPORT CartographerExport
  DESTINATION share/cartographer/cmake/
  FILE CartographerTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  DESTINATION share/cartographer/
)
