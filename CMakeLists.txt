cmake_minimum_required(VERSION 2.8.3)
project(lk_rtabmap_zed)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
              roscpp eigen_conversions pcl_conversions
  )

## System dependencies are found with CMake's conventions
find_package(RTABMap 0.12.3 REQUIRED)

find_package(OpenCV REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lk_rtabmap_zed_ros
  CATKIN_DEPENDS roscpp eigen_conversions pcl_conversions
  DEPENDS RTABMap OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${RTABMap_LIBRARIES}
)

add_executable(lk_rtabmap_zed_node src/WrapperNode.cpp)
target_link_libraries(lk_rtabmap_zed_node ${Libraries})

add_dependencies(lk_rtabmap_zed_node ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(lk_rtabmap_zed_ros_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(lk_rtabmap_zed_ros_wrapper_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lk_rtabmap_zed_ros_wrapper lk_rtabmap_zed_ros_wrapper_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lk_rtabmap_zed_ros_wrapper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
