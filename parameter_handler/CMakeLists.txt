cmake_minimum_required(VERSION 2.8.3)
project(parameter_handler)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin QUIET)

if(catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS
    message_logger
  )
else()
  find_package(message_logger REQUIRED)
endif()

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES parameter_handler
    CATKIN_DEPENDS message_logger
  #  DEPENDS system_lib
  )
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
else()
  include_directories(
    ${message_logger_INCLUDE_DIRS}
  )
endif()

add_library(parameter_handler SHARED
  src/parameter_handler.cpp
)

if(catkin_FOUND)
  add_dependencies(parameter_handler
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(parameter_handler
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(test_parameter_handler
    test/test_main.cpp
    test/Parameter_tests.cpp
  )

  add_dependencies(test_parameter_handler
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(test_parameter_handler
    parameter_handler
    ${catkin_LIBRARIES}
  )
else()
  add_dependencies(parameter_handler
    message_logger
  )

  target_link_libraries(parameter_handler
    ${message_logger_LIBRARIES}
  )
endif()

if(catkin_FOUND)
  install(TARGETS parameter_handler
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()

### Local Build ###

if(NOT catkin_FOUND)
  set(external_libs
    ${message_logger_LIBRARIES}
  )

  # Register the local build in case on doesn't use "make install"
  export(PACKAGE parameter_handler)

  # Create variable with the library location
  set(LIB_NAME parameter_handler)
  set(parameter_handler_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(parameter_handler_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(parameter_handlerConfig.cmake.in
    "${PROJECT_BINARY_DIR}/parameter_handlerConfig.cmake" @ONLY
  )
endif()
