cmake_minimum_required(VERSION 2.8.3)
project(leg_detector)

# look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

# Look for Bullet
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  std_srvs
  sensor_msgs
  laser_geometry
  tf
  visualization_msgs
  people_msgs
  people_tracking_filter
  image_geometry
  rosbag
  dynamic_reconfigure
)

## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/LegDetector.cfg
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS people_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs)

## Declare a cpp executable
add_executable(leg_detector 
               src/laser_processor.cpp
               src/leg_detector.cpp 
               src/calc_leg_features.cpp)

target_link_libraries(leg_detector
   ${catkin_LIBRARIES} ${BFL_LIBRARIES} ${BULLET_LIBRARIES}
)

add_executable(train_people_detector 
               src/train_people_detector.cpp
               src/laser_processor.cpp
               src/calc_leg_features.cpp)

target_link_libraries(train_people_detector
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
add_dependencies(leg_detector people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(train_leg_detector ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(train_leg_detector ${PROJECT_NAME}_gencpp)
add_dependencies(train_leg_detector ${PROJECT_NAME}_gencfg)


install(TARGETS
    leg_detector
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
