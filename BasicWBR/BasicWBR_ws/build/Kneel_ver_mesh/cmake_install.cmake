# Install script for directory: /home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/Kneel_ver_mesh/catkin_generated/installspace/Kneel_ver_mesh.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh/cmake" TYPE FILE FILES
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/Kneel_ver_mesh/catkin_generated/installspace/Kneel_ver_meshConfig.cmake"
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/Kneel_ver_mesh/catkin_generated/installspace/Kneel_ver_meshConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh/config" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh/launch" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh/meshes" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/Kneel_ver_mesh/urdf" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/Kneel_ver_mesh/urdf/")
endif()

