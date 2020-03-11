cmake_minimum_required(VERSION 3.4)
project(pandacontroller CXX)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS  "-O3 -fopenmp")
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ../lib)
SET( CMAKE_INSTALL_PREFIX ../../../devel )

find_library(franka libfranka.so.0.7.1 REQUIRED)
find_package(Eigen3 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)
message(STATUS ${franka})
message(STATUS ${DHDC})
message(STATUS ${DRDC})

add_library(common SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/Common.cpp)
target_link_libraries(common PUBLIC franka)
target_include_directories(common PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(PandaController SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/PandaController.cpp)
target_link_libraries(PandaController PUBLIC common franka)
target_include_directories(PandaController PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(panda_controller_ext SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/panda_controller_ext.cpp)
target_link_libraries(panda_controller_ext PUBLIC PandaController common franka)
target_include_directories(panda_controller_ext PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(Falcon ${CMAKE_CURRENT_SOURCE_DIR}/src/Falcon.cpp)
target_link_libraries(Falcon franka ${LIBNIFALCON} PandaController)
target_include_directories(Falcon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ForceDimension ${CMAKE_CURRENT_SOURCE_DIR}/src/ForceDimension.cpp)
target_link_libraries(ForceDimension franka ${LIBNIFALCON} PandaController ${DRDC} ${DHDC})
target_include_directories(ForceDimension PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(ForceDimensionClutch ${CMAKE_CURRENT_SOURCE_DIR}/src/ForceDimension_clutch.cpp)
target_link_libraries(ForceDimensionClutch franka ${LIBNIFALCON} PandaController ${DRDC} ${DHDC})
target_include_directories(ForceDimensionClutch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


message(${CMAKE_INSTALL_PREFIX})
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS PandaController Falcon ForceDimension ForceDimensionClutch common
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION ".")
