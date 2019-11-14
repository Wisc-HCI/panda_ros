cmake_minimum_required(VERSION 3.4)
project(falcon_ros CXX)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Release)
SET(relaxed_ik_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/share/relaxed_ik/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg relaxed_ik)
find_package(Eigen3 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
#add_message_files(DIRECTORY msg FILES <>)
#generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS relaxed_ik)

add_executable(rosFalcon ${CMAKE_CURRENT_SOURCE_DIR}/src/rosFalcon.cpp)
target_link_libraries(rosFalcon ${LIBNIFALCON}  ${catkin_LIBRARIES})
target_include_directories(rosFalcon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)

install(TARGETS rosFalcon
  RUNTIME DESTINATION lib/falcon_ros
)
