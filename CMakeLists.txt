cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME occ-map)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

pods_install_headers(src/PixelMap.hpp src/PixelMap.hxx
					src/VoxelMap.hpp src/VoxelMap.hxx
					DESTINATION occ_map)


pods_install_pkg_config_file(occ-map
    LIBS -lz
    REQUIRES lcm lcmtypes_occ-map
    VERSION 0.0.1)

add_subdirectory(src/renderers)
add_subdirectory(src/test)
add_subdirectory(src/map_server)
