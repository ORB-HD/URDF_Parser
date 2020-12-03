CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(urdfparser)

FIND_PACKAGE(Boost REQUIRED)

SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/tinyxml/include
  ${BOOST_INCLUDE_DIR}
)

OPTION(RUN_TEST "enable testing of library" OFF)

ADD_SUBDIRECTORY(
  tinyxml
)

SET( URDF_SRCS
  src/common.cpp
  src/joint.cpp
  src/geometry.cpp
  src/link.cpp
  src/model.cpp
)

ADD_LIBRARY( urdfparser STATIC
  ${URDF_SRCS}
)

SET_PROPERTY(TARGET urdfparser PROPERTY POSITION_INDEPENDENT_CODE ON)

TARGET_LINK_LIBRARIES( urdfparser
  mytinyxml
 )

IF(RUN_TEST)
  FIND_PACKAGE(Catch2 REQUIRED)
  ADD_EXECUTABLE(test test/parse_simple.cpp)
  TARGET_LINK_LIBRARIES(test
    Catch2::Catch2
    urdfparser
  )
ENDIF(RUN_TEST)
