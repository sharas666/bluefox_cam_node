# Install script for directory: /home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/josh/uni/project/bluefox_cam_node/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/bluefox_cam_node" TYPE FILE FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/include/bluefox_cam_node/bluefox_cam_nodeConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/bluefox_cam_node" TYPE FILE FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/python2.7/dist-packages/bluefox_cam_node/__init__.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/python2.7/dist-packages/bluefox_cam_node/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/bluefox_cam_node" TYPE DIRECTORY FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/python2.7/dist-packages/bluefox_cam_node/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node/catkin_generated/installspace/bluefox_cam_node.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluefox_cam_node/cmake" TYPE FILE FILES
    "/home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node/catkin_generated/installspace/bluefox_cam_nodeConfig.cmake"
    "/home/josh/uni/project/bluefox_cam_node/catkin_ws/build/bluefox_cam_node/catkin_generated/installspace/bluefox_cam_nodeConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/bluefox_cam_node" TYPE FILE FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/src/bluefox_cam_node/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node" TYPE EXECUTABLE FILES "/home/josh/uni/project/bluefox_cam_node/catkin_ws/devel/lib/bluefox_cam_node/bluefox_cam_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node")
    file(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bluefox_cam_node/bluefox_cam_node")
    endif()
  endif()
endif()

