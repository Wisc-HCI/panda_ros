cmake_minimum_required(VERSION 3.0)
project(panda_ros)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs genmsg)
catkin_package()

find_library(franka libfranka.so.0.7.1 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
message(STATUS ${pandaCommon})

add_executable(pandaRosWrapper src/panda_ros.cpp)
target_include_directories(pandaRosWrapper PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(pandaRosWrapper ${catkin_LIBRARIES} ${pandaController} ${pandaCommon} franka)

install(TARGETS pandaRosWrapper
  RUNTIME DESTINATION lib/panda_ros
)
