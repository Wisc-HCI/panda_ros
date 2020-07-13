cmake_minimum_required(VERSION 3.0)
project(dmp_deformations)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp rospy tf2_ros geometry_msgs sensor_msgs std_msgs genmsg)
catkin_package()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)
find_library(franka libfranka.so.0.7.1 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)

message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
message(STATUS ${pandaCommon})


if(DHDC)
  if(DRDC)
    add_executable(forcedimensionDMP src/ForceDimension_DMP_ros.cpp)
    target_include_directories(forcedimensionDMP PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
    target_link_libraries(forcedimensionDMP ${catkin_LIBRARIES} ${pandaController} ${pandaCommon} franka ${DRDC} ${DHDC})

    install(TARGETS forcedimensionDMP
      RUNTIME DESTINATION lib/forcedimensionDMP
    )
  endif()
endif()
