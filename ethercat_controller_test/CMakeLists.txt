cmake_minimum_required(VERSION 2.8.3)
project(ethercat_controller_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros rtt_roscomm)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

install(FILES 
  test.launch 
  test.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_definitions(-DRTT_COMPONENT)
orocos_component(${PROJECT_NAME} src/AmcTest.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package()

