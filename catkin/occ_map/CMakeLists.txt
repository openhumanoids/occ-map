cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

project(occ_map)

find_package(catkin REQUIRED COMPONENTS
  occ_map_lcmtypes
)

catkin_package(
  INCLUDE_DIRS ../../src
#  LIBRARIES occ_map
  CATKIN_DEPENDS occ_map_lcmtypes
#  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  ../../src
  ${catkin_INCLUDE_DIRS}
)

add_executable(vox-map-test ../../src/test/VoxelMapTest.cpp)
target_link_libraries (vox-map-test ${catkin_LIBRARIES} z)

add_executable(pix-map-test ../../src/test/PixelMapTest.cpp)
target_link_libraries (pix-map-test ${catkin_LIBRARIES} z)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})