cmake_minimum_required(VERSION 2.8.3)
project(husky_ur_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
