
cmake_minimum_required( VERSION 3.13 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "build type" FORCE)
endif()

project( absolute_pose_from_oriented_and_scaled_features )

set (CMAKE_CXX_STANDARD 11)

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( PoseLib REQUIRED )

include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(p2ori)
add_subdirectory(up1sift)
add_subdirectory(affine2sift)
add_subdirectory(examples)

