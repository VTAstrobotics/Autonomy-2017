cmake_minimum_required(VERSION 3.5)
project(FieldMapping CXX)

# add source for any additional modules here
set(LIB_SOURCE
  AutonomyMap.h 
  PathFinder.h
  Map.h Map.cpp
  Move.h Move.cpp
  Path.h Path.cpp
  Robot.h Robot.cpp)

# add source for FieldMapping program here
set(FIELDMAPPING_SOURCE PathFinder.cpp)

SET(OBJS
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/obj.o
)

SET_SOURCE_FILES_PROPERTIES(
  ${OBJS}
  PROPERTIES
  EXTERNAL_OBJECT true
  GENERATED true
)

# try to prevent accidental in-source builds, these cause lots of problems
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Remove any files created thus far and use a different directory for the build.")
endif()

# require a C++11 compiler for all targets
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(UNIX AND STRICT)
  message("-- Enabling strict compilation mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
endif()

add_executable(PathFinder ${FIELDMAPPING_SOURCE} ${LIB_SOURCE})

