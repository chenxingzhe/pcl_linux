project(pcltest)

add_executable(pcltest main.cpp)
 find_package(OpenCV REQUIRED)  
 target_link_libraries(pcltest ${OpenCV_LIBS})
#openni2
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})
include_directories(${OpenNI2_INCLUDEDIR} )
target_link_libraries(pcltest  ${OpenNI2_LIBRARIES} )
#PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(pcltest ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
#boost
#find_package(Boost REQUIRED)

#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})
#add_definitions(${Boost_DEFINITIONS})
#VTK
#set(VTK_DIR "/usr/local/lib/vtk-5.10")
#set(VTK_INCLUDE_DIRS "/usr/local/include/vtk-5.10")
#include(${VTK_USE_FILE})
#include_directories(${VTK_INCLUDE_DIRS})
#target_link_libraries(opennitest  ${VTK_DIR} )
#add_definitions(${VTK_DEFINITIONS})
#flann
#find_package(FLANN REQUIRED)

#include_directories(${FLANN_INCLUDE_DIRS})
#link_directories(${FLANN_LIBRARY_DIRS})
#add_definitions(${FLANN_DEFINITIONS})
#EIGEN
#find_package(EIGEN REQUIRED)
#set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
#include_directories(${EIGEN_INCLUDE_DIRS})
#link_directories(${EIGEN_LIBRARY_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
#OPENNI
include_directories(${OPENNI_INCLUDE_DIRS})
link_directories(${OPENNI_LIBRARY_DIRS})
add_definitions(${OPENNI_DEFINITIONS})
#install(TARGETS opennitest RUNTIME DESTINATION bin)

