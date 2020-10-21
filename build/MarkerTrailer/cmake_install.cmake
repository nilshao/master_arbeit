<<<<<<< HEAD
# Install script for directory: /home/sibohao/Desktop/master_arbeit/src/MarkerTrailer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sibohao/Desktop/master_arbeit/install")
=======
# Install script for directory: /home/zmc/Desktop/master_arbeit/src/MarkerTrailer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zmc/Desktop/master_arbeit/install")
>>>>>>> dd90c5f2991f6a92ee7d6222f11eecb6dd07d55e
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
<<<<<<< HEAD
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
=======
    set(CMAKE_INSTALL_CONFIG_NAME "")
>>>>>>> dd90c5f2991f6a92ee7d6222f11eecb6dd07d55e
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
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sibohao/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailer.pc")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zmc/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailer.pc")
>>>>>>> dd90c5f2991f6a92ee7d6222f11eecb6dd07d55e
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/MarkerTrailer/cmake" TYPE FILE FILES
<<<<<<< HEAD
    "/home/sibohao/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailerConfig.cmake"
    "/home/sibohao/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailerConfig-version.cmake"
=======
    "/home/zmc/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailerConfig.cmake"
    "/home/zmc/Desktop/master_arbeit/build/MarkerTrailer/catkin_generated/installspace/MarkerTrailerConfig-version.cmake"
>>>>>>> dd90c5f2991f6a92ee7d6222f11eecb6dd07d55e
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
<<<<<<< HEAD
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/MarkerTrailer" TYPE FILE FILES "/home/sibohao/Desktop/master_arbeit/src/MarkerTrailer/package.xml")
=======
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/MarkerTrailer" TYPE FILE FILES "/home/zmc/Desktop/master_arbeit/src/MarkerTrailer/package.xml")
>>>>>>> dd90c5f2991f6a92ee7d6222f11eecb6dd07d55e
endif()

