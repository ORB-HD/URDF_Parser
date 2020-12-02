CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(urdfparser)

FIND_PACKAGE(Boost REQUIRED)

SET(CMAKE_CXX_STANDARD 17)

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/tinyxml/include
  ${BOOST_INCLUDE_DIR}
)

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
  tinyxml
 )
