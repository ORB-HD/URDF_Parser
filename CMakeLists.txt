CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(urdfparser)

SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

OPTION(URDF_BUILD_TEST "enable testing of library" OFF)

SET( URDF_SRCS
  src/common.cpp
  src/joint.cpp
  src/geometry.cpp
  src/link.cpp
  src/model.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
)

ADD_LIBRARY( urdfparser STATIC
  ${URDF_SRCS}
)

SET(URDFPARSER_LIB urdfparser)

SET_PROPERTY(TARGET urdfparser PROPERTY POSITION_INDEPENDENT_CODE ON)

IF(URDF_BUILD_TEST)
  FIND_PACKAGE(Catch2 REQUIRED)
  ADD_EXECUTABLE(test_library test/parse_simple.cpp)
  TARGET_LINK_LIBRARIES(test_library
    Catch2::Catch2
    urdfparser
  )
ENDIF(URDF_BUILD_TEST)
