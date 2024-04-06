# Install script for directory: /home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/wbrbodytest/catkin_generated/installspace/wbrbodytest.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest/cmake" TYPE FILE FILES
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/wbrbodytest/catkin_generated/installspace/wbrbodytestConfig.cmake"
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/build/wbrbodytest/catkin_generated/installspace/wbrbodytestConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest/config" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest/launch" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest/meshes" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/wbrbodytest/urdf" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicWBR/BasicWBR_ws/src/wbrbodytest/urdf/")
endif()

