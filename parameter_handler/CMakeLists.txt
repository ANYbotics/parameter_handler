cmake_minimum_required(VERSION 3.5.1)
project(parameter_handler)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
  tinyxml_tools
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Use boost_pfr as optional dependency, only add helpers if boost_pfr is found and CMAKE_CXX_STANDARD is defined 14 or higher.
include(cmake/${PROJECT_NAME}-extras.cmake)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/parameter_handler.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  catkin_add_gmock(test_${PROJECT_NAME}
    test/test_main.cpp
    test/Parameter_tests.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${GTEST_BOTH_LIBRARIES}
      gmock_main gmock gtest pthread
      ${PROJECT_NAME}
      )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage(
        TEST_BUILD_TARGETS test_${PROJECT_NAME}
      )
    endif(cmake_code_coverage_FOUND)
  endif()
endif()

#################
## Clang Tools ##
#################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(TARGETS ${PROJECT_NAME})
endif(cmake_clang_tools_FOUND)

