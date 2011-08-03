add_executable(occ-map-pixmap-saver pixmap_saver.cpp)
pods_use_pkg_config_packages(occ-map-pixmap-saver occ-map)   
pods_install_executables(occ-map-pixmap-saver) 

add_executable(occ-map-pixmap-server pixmap_server.cpp)
pods_use_pkg_config_packages(occ-map-pixmap-server occ-map)   
pods_install_executables(occ-map-pixmap-server) 
