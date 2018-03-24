project(pcl_plugin)
cmake_minimum_required(VERSION 2.8)

option(BUILD_TESTING "Build with tests enabled." ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(USE_OPENNI "Build with the OpenNI filters, required PCL OpenNI support." OFF)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")

find_package(catkin  REQUIRED COMPONENTS pcl_ros)

find_package(VTK REQUIRED)

catkin_package(
)

include(${VTK_USE_FILE})


include_directories(
  Filters
#  ${catkin_INCLUDE_DIRS}
)


add_subdirectory(Filters)


if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
