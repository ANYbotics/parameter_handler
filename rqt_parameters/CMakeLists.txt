cmake_minimum_required(VERSION 2.8.3)
project(rqt_parameters)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    parameter_handler_msgs
)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

find_package(OpenGL REQUIRED)

catkin_python_setup()



###########
# sources #
###########
set(rqt_parameters_SRCS
    src/ParametersPlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_parameters_HDRS
    include/rqt_parameters/ParametersPlugin.hpp
    include/rqt_parameters/DoubleParameter.hpp
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
    LIBRARIES rqt_parameters
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp parameter_handler_msgs
)


qt4_wrap_cpp(rqt_parameters_MOCS ${rqt_parameters_HDRS})
qt4_wrap_ui(rqt_parameters_UIS_H ${rqt_parameters_UIS})


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
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${OPENGL_LIBRARIES}
    glut GL GLU GLEW
)


find_package(class_loader)
class_loader_hide_library_symbols(rqt_parameters)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS rqt_parameters
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(PROGRAMS scripts/rqt_parameters
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_parameters
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
