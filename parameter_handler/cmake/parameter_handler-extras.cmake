# optional include of boost_pfr for all dependants of parameter_handler.
find_package(boost_pfr QUIET)

if(boost_pfr_FOUND)
  if(DEFINED CMAKE_CXX_STANDARD)
    if(${CMAKE_CXX_STANDARD} GREATER_EQUAL 14)
      message("Building parameter_handler with boost_pfr")
      add_definitions(-DUSE_BOOST_PFR)
      list(APPEND catkin_INCLUDE_DIRS ${boost_pfr_INCLUDE_DIRS})
      list(APPEND catkin_LIBRARIES ${boost_pfr_LIBRARIES})
    endif ()
  endif ()
endif ()