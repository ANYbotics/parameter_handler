cmake_minimum_required(VERSION 3.16.3)
project(parameter_handler)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # gersemi: off


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



  # gersemi: on
else() # ROS version 2
  ###############
  ## Configure ##
  ###############

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_CXX_STANDARD 17)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-gnu-zero-variadic-macro-arguments
  ) # -Wno-gnu-zero-variadic-macro-arguments required for gtest pre-c++20 with -Wpedantic

  set(
    PACKAGE_DEPENDENCIES
    boost_pfr
    tinyxml_tools
    message_logger
    Eigen3
  )

  find_package(ament_cmake REQUIRED)
  foreach(PKG ${PACKAGE_DEPENDENCIES})
    find_package(${PKG} REQUIRED)
  endforeach()

  # non-modern cmake definitions. For compatibility with the way this pkg is used.
  add_definitions(-DUSE_BOOST_PFR)

  add_library(${PROJECT_NAME} SHARED src/parameter_handler.cpp)
  target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

  install(
    DIRECTORY
    include/
    DESTINATION
    include/${PROJECT_NAME}
  )

  install(
    TARGETS
    ${PROJECT_NAME}
    EXPORT
    export_${PROJECT_NAME}
    LIBRARY
    DESTINATION
    lib
    ARCHIVE
    DESTINATION
    lib
    RUNTIME
    DESTINATION
    bin
  )

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(${PACKAGE_DEPENDENCIES})

  ##########
  ## Test ##
  ##########
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_gmock REQUIRED)
    ament_add_gmock(
      test_${PROJECT_NAME}
      test/test_main.cpp
      test/Parameter_tests.cpp
      SKIP_LINKING_MAIN_LIBRARIES
    )
    target_include_directories(test_${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

    # cannot put PRIVATE here as ament_add_gmock already messed with the linked libraries
    # https://stackoverflow.com/questions/59522267/cmake-rejects-a-second-target-link-libraries-talking-about-keyword-vs-plain
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

    ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
  endif()

  ament_package()
endif()
