cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
              roco
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parameter_handler
  CATKIN_DEPENDS 
     roco
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(parameter_handler
  src/parameter_handler.cpp
)

add_dependencies(parameter_handler ${catkin_EXPORTED_TARGETS})

target_link_libraries(parameter_handler
   ${catkin_LIBRARIES}
)


catkin_add_gtest(test_parameter_handler
    test/test_main.cpp
    test/Parameter_tests.cpp
)
add_dependencies(test_parameter_handler ${catkin_EXPORTED_TARGETS})


target_link_libraries(test_parameter_handler
    parameter_handler
    ${catkin_LIBRARIES}
)