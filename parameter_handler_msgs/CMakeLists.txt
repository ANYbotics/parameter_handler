cmake_minimum_required(VERSION 3.5.1)
project(parameter_handler_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES
  std_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_generation
)

add_message_files(
  FILES
    FloatingPointParameter.msg
    IntegralParameter.msg
)

add_service_files(
  FILES
    SetIntegralParameter.srv
    SetFloatingPointParameter.srv
    GetIntegralParameter.srv
    GetFloatingPointParameter.srv
    GetParameterList.srv
)

generate_messages(
  DEPENDENCIES
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)
