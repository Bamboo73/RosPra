# Install script for directory: /home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/src/turtlecmdpkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/build/turtlecmdpkg/catkin_generated/installspace/turtlecmdpkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlecmdpkg/cmake" TYPE FILE FILES
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/build/turtlecmdpkg/catkin_generated/installspace/turtlecmdpkgConfig.cmake"
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/build/turtlecmdpkg/catkin_generated/installspace/turtlecmdpkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlecmdpkg" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/ComTopic/ComTopic_ws/src/turtlecmdpkg/package.xml")
endif()

