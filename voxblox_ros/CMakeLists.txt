cmake_minimum_required(VERSION 3.0.2)
project(voxblox_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_definitions(-std=c++14 -Wall -Wextra)

############
# PROTOBUF #
############
set(PROTOBUF_ROOT "/home/december/third_party_libraries/protobuf")
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROTOBUF_ROOT})
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
link_directories(${PROTOBUF_ROOT}/lib)
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/esdf_server.cc
  src/interactive_slider.cc
  src/simulation_server.cc
  src/intensity_server.cc
  src/traversability_tsdf_server.cc
  src/transformer.cc
  src/tsdf_server.cc
  src/height_server.cc
)

############
# BINARIES #
############

cs_add_executable(voxblox_eval
  src/voxblox_eval.cc
)
target_link_libraries(voxblox_eval ${PROJECT_NAME})

cs_add_executable(tsdf_server
  src/tsdf_server_node.cc
)
target_link_libraries(tsdf_server ${PROJECT_NAME})

cs_add_executable(esdf_server
  src/esdf_server_node.cc
)
target_link_libraries(esdf_server ${PROJECT_NAME})

cs_add_executable(intensity_server
  src/intensity_server_node.cc
)
target_link_libraries(intensity_server ${PROJECT_NAME})

cs_add_executable(traversability_tsdf_server
        src/traversability_tsdf_server_node.cc
)
target_link_libraries(traversability_tsdf_server ${PROJECT_NAME})

cs_add_executable(height_server
        src/height_server_node.cc
)
target_link_libraries(height_server ${PROJECT_NAME})

cs_add_executable(simulation_eval
  src/simulation_eval.cc
)
target_link_libraries(simulation_eval ${PROJECT_NAME})

cs_add_executable(visualize_tsdf
  src/visualize_tsdf.cc
)
target_link_libraries(visualize_tsdf ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
