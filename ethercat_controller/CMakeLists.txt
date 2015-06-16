cmake_minimum_required(VERSION 2.8.3)
project(ethercat_controller)

find_package(catkin REQUIRED COMPONENTS rtt_ros soem cmake_modules)

find_package(Eigen REQUIRED)

find_package(Xenomai REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${Xenomai_INCLUDE_DIR})

if(${OROCOS_TARGET} STREQUAL "xenomai" )
orocos_component(ecmaster src/ecmaster.cpp src/amc_driver.cpp src/amc_subnode.cpp)
target_link_libraries(ecmaster ${catkin_LIBRARIES} soem ${Xenomai_RTDM_LIBRARIES})
endif()

orocos_generate_package()


