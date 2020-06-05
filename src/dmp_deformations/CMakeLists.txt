cmake_minimum_required(VERSION 3.0)
project(dmp_deformations)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp rospy tf2_ros geometry_msgs sensor_msgs std_msgs genmsg relaxed_ik)
catkin_package()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)
find_library(franka libfranka.so.0.7.1 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
find_library(BSpline libBSpline.so REQUIRED)
find_library(NLopt libnlopt.so REQUIRED)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)

add_executable(forcedimensionDMP src/ForceDimension_DMP_ros.cpp)
target_include_directories(forcedimensionDMP PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(forcedimensionDMP ${catkin_LIBRARIES} ${pandaController} ${pandaCommon} franka ${DRDC} ${DHDC})

add_executable(falconDMP src/Falcon_DMP_ros.cpp)
target_include_directories(falconDMP PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(falconDMP ${catkin_LIBRARIES} ${LIBNIFALCON} ${pandaController} ${pandaCommon} franka ${DRDC} ${DHDC})

add_executable(falconVREPDMP src/Falcon_DMP_VREP_ros.cpp)
target_include_directories(falconVREPDMP PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(falconVREPDMP ${catkin_LIBRARIES} ${LIBNIFALCON} ${pandaController} ${pandaCommon} franka ${DRDC} ${DHDC} ${BSpline} ${NLopt})

add_executable(FDdeformations src/ForceDimension_Deformation_Emmanuel.cpp)
target_include_directories(FDdeformations PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(FDdeformations ${catkin_LIBRARIES} ${DRDC} ${DHDC})

install(TARGETS FDdeformations
  RUNTIME DESTINATION lib/FDdeformations
)

install(TARGETS falconDMP
  RUNTIME DESTINATION lib/falconDMP
)

install(TARGETS forcedimensionDMP
  RUNTIME DESTINATION lib/forcedimensionDMP
)

install(TARGETS falconVREPDMP
  RUNTIME DESTINATION lib/falconVREPDMP
)
