cmake_minimum_required(VERSION 2.8.3)
project(bluefox_cam_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-narrowing -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport cv_bridge dynamic_reconfigure
    )
find_package(mvIMPACT REQUIRED)
find_package( OpenCV REQUIRED )

generate_dynamic_reconfigure_options( cfg/bluefox_cam_node.cfg)


add_library(mvStereoVisionCamera
  src/mvStereoVision/src/Camera.cpp
)
add_library(mvStereoSystem
  src/mvStereoVision/src/Stereosystem.cpp
)
target_link_libraries (mvStereoVisionCamera ${mvIMPACT_LIBRARIES} ${OpenCV_LIBRARIES}
)
add_library(mvStereoVisionUtillity
  src/mvStereoVision/src/utility.cpp
)
add_executable(bluefox_cam_node src/bluefox_cam_node.cpp)
add_dependencies(bluefox_cam_node ${PROJECT_NAME}_gencfg)

target_link_libraries (bluefox_cam_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} mvStereoVisionUtillity mvStereoVisionCamera mvStereoSystem)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES bluefox_cam_node
    #  DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${mvIMPACT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
