cmake_minimum_required(VERSION 3.0)
project(panda_ros)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs genmsg)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package()

find_library(
    pandaController
    NAMES PandaController
    
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
message(STATUS ${pandaCommon})

add_executable(rosPandaVelocity src/main.cpp)
target_include_directories(rosPandaVelocity PUBLIC ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(rosPandaVelocity ${catkin_LIBRARIES} ${pandaController} ${pandaCommon})

install(TARGETS rosPandaVelocity
  RUNTIME DESTINATION lib/panda_ros
)
