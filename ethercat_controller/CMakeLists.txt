cmake_minimum_required(VERSION 2.8.3)
project(ethercat_controller)

find_package(catkin REQUIRED COMPONENTS rtt_ros soem cmake_modules)

find_package(Eigen REQUIRED)

find_package(Xenomai REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(ecmaster src/ecmaster.cpp src/AxisDrive.cpp src/AxisDrive_debug.cpp)
target_link_libraries(ecmaster ${catkin_LIBRARIES} soem ${Xenomai_RTDM_LIBRARIES})

orocos_generate_package()

#Start of user code user_build
#if(${OROCOS_TARGET} STREQUAL "xenomai" )
# message("Found RTNET: ${RTNET_INCLUDE_DIR}")
# include_directories(${RTNET_INCLUDE_DIR})
#  add_definitions(-DHAVE_RTNET)
  # try Xenomai
#  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}" -DXENOMAI)
#  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  #orocos_add_compile_flags(${PROJECT_NAME} ${XENO_CFLAGS})
  #orocos_add_link_flags(${PROJECT_NAME} ${XENO_LDFLAGS})
  #TARGET_LINK_LIBRARIES(${PROJECT_NAME} "-lrtdm")
  
#  orocos_add_compile_flags(ecmaster ${XENO_CFLAGS})
#  orocos_add_link_flags(ecmaster ${XENO_LDFLAGS})
#  TARGET_LINK_LIBRARIES(ecmaster "-lrtdm")

#endif()
#End of user code

