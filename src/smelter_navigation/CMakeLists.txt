cmake_minimum_required(VERSION 3.8)
project(smelter_navigation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config rviz DESTINATION share/${PROJECT_NAME})
set(MAP_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../doc/maps")
install(FILES "${MAP_SOURCE}/jueying.pgm" "${MAP_SOURCE}/jueying.yaml"
  DESTINATION share/${PROJECT_NAME}/maps)
ament_package()
