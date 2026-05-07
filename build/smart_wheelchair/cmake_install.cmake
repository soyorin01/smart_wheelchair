# Install script for directory: /home/soyorin/wheelchair_ws/src/smart_wheelchair

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/soyorin/wheelchair_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/soyorin/wheelchair_ws/build/smart_wheelchair/catkin_generated/installspace/smart_wheelchair.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smart_wheelchair/cmake" TYPE FILE FILES
    "/home/soyorin/wheelchair_ws/build/smart_wheelchair/catkin_generated/installspace/smart_wheelchairConfig.cmake"
    "/home/soyorin/wheelchair_ws/build/smart_wheelchair/catkin_generated/installspace/smart_wheelchairConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smart_wheelchair" TYPE FILE FILES "/home/soyorin/wheelchair_ws/src/smart_wheelchair/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/smart_wheelchair" TYPE PROGRAM FILES "/home/soyorin/wheelchair_ws/build/smart_wheelchair/catkin_generated/installspace/teleop_keyboard.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/smart_wheelchair" TYPE DIRECTORY FILES
    "/home/soyorin/wheelchair_ws/src/smart_wheelchair/launch/"
    "/home/soyorin/wheelchair_ws/src/smart_wheelchair/worlds/"
    "/home/soyorin/wheelchair_ws/src/smart_wheelchair/maps/"
    "/home/soyorin/wheelchair_ws/src/smart_wheelchair/param/"
    "/home/soyorin/wheelchair_ws/src/smart_wheelchair/config/"
    )
endif()

