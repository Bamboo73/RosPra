# Install script for directory: /home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/src/basic_lqr_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/basic_lqr_pkg" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/devel/include/basic_lqr_pkg/dyparaConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/basic_lqr_pkg" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/devel/lib/python3/dist-packages/basic_lqr_pkg/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/devel/lib/python3/dist-packages/basic_lqr_pkg/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/basic_lqr_pkg" TYPE DIRECTORY FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/devel/lib/python3/dist-packages/basic_lqr_pkg/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/build/basic_lqr_pkg/catkin_generated/installspace/basic_lqr_pkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_lqr_pkg/cmake" TYPE FILE FILES
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/build/basic_lqr_pkg/catkin_generated/installspace/basic_lqr_pkgConfig.cmake"
    "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/build/basic_lqr_pkg/catkin_generated/installspace/basic_lqr_pkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/basic_lqr_pkg" TYPE FILE FILES "/home/bamboo70/MyDocs/ROSPra/ReviewFiles/BasicLQR/basicwip_ws/src/basic_lqr_pkg/package.xml")
endif()
