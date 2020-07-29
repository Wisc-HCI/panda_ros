cmake_minimum_required(VERSION 3.0)
project(inputs_ros)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)
SET(CATKIN_PACKAGE_BIN_DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib)

SET(relaxed_ik_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/share/relaxed_ik/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp tf tf2_ros rospy std_msgs genmsg relaxed_ik)
find_package(Eigen3 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
message(STATUS HERE)
message(STATUS ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
    nodes/space_mouse
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS relaxed_ik)


if(LIBNIFALCON)
  add_executable(falconRosWrapper ${CMAKE_CURRENT_SOURCE_DIR}/src/falcon_ros.cpp)
  target_link_libraries(falconRosWrapper ${LIBNIFALCON}  ${catkin_LIBRARIES})
  target_include_directories(falconRosWrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)

  install(TARGETS falconRosWrapper
    RUNTIME DESTINATION lib/falcon_ros
  )
endif()

if(DHDC)
  if(DRDC)
    add_executable(forcedimensionRosWrapper ${CMAKE_CURRENT_SOURCE_DIR}/src/forcedimension_ros.cpp)
    target_link_libraries(forcedimensionRosWrapper ${LIBNIFALCON}  ${catkin_LIBRARIES} ${DRDC} ${DHDC})
    target_include_directories(forcedimensionRosWrapper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)

    install(TARGETS forcedimensionRosWrapper
      RUNTIME DESTINATION lib/forcedimension_ros
    )
  endif()
endif()
