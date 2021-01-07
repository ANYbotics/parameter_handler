cmake_minimum_required(VERSION 3.5.1)
project(parameter_handler_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    parameter_handler
    parameter_handler_ros
    parameter_handler_std
    parameter_handler_msgs
    rqt_parameters
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      parameter_handler
      parameter_handler_ros
      parameter_handler_std
      parameter_handler_msgs
      rqt_parameters
    DEPENDS
  )

  # This is important to resolve the symlink of this doc package and allows to refer to packages in subfolders and READMEs outside of a package.
  get_filename_component(REAL_PACKAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/" REALPATH)

  # Set some environment variables
  set(DOXYGEN_SOURCE_DIRS "\"${REAL_PACKAGE_PATH}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../parameter_handler/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../parameter_handler_msgs/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../parameter_handler_ros/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../parameter_handler_std/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${REAL_PACKAGE_PATH}/../rqt_parameters/\"")
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${REAL_PACKAGE_PATH}/figures/\"" )
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen.
  add_custom_target(
    ${PROJECT_NAME} ALL
    COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config
    WORKING_DIRECTORY ${DOCSOURCE_DIR}
    COMMENT "${PROJECT_NAME}: Generating API documentation."
    VERBATIM
  )
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${DOCSOURCE_DIR} CACHE)
  unset(${DOXYGEN_IMAGE_DIRS} CACHE)
  unset(${HTML_DIR} CACHE)
  unset(${REAL_PACKAGE_PATH} CACHE)


ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
