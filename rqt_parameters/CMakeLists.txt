cmake_minimum_required(VERSION 2.8.3)
project(rqt_parameters)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    parameter_handler_msgs
    std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()



###########
# sources #
###########
set(rqt_parameters_SRCS
    src/rqt_parameters/ParametersPlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_parameters_HDRS
    include/rqt_parameters/ParametersPlugin.hpp
    include/rqt_parameters/MatrixSpinBox.hpp
    include/rqt_parameters/ParameterBase.hpp
    include/rqt_parameters/ParameterFloat64Matrix.hpp
    include/rqt_parameters/ParameterInt32Matrix.hpp
    include/rqt_parameters/multiarray_helpers.hpp
)
###########



############
# ui files #
############
set(rqt_parameters_UIS
    resource/parameters_plugin.ui
)
############



set(rqt_parameters_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
    INCLUDE_DIRS ${rqt_parameters_INCLUDE_DIRECTORIES}
    LIBRARIES rqt_parameters
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp parameter_handler_msgs std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_parameters_MOCS ${rqt_parameters_HDRS})
  qt5_wrap_ui(rqt_parameters_UIS_H ${rqt_parameters_UIS})
else()
  qt4_wrap_cpp(rqt_parameters_MOCS ${rqt_parameters_HDRS})
  qt4_wrap_ui(rqt_parameters_UIS_H ${rqt_parameters_UIS})
endif()


include_directories(
    ${rqt_parameters_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)


add_library(rqt_parameters
    ${rqt_parameters_SRCS}
    ${rqt_parameters_MOCS}
    ${rqt_parameters_UIS_H}
)

add_dependencies(rqt_parameters ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_parameters
    ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_parameters)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_parameters
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_parameters
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
