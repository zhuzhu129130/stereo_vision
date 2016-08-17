cmake_minimum_required(VERSION 2.6)
project(irdv_sim)

find_package(OpenCV REQUIRED )
find_package(OpenGL REQUIRED )
find_package(GLUT REQUIRED)

# 增加PCL库的依赖
FIND_PACKAGE( PCL 1.8 REQUIRED COMPONENTS common io visualization filters )

include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${OpenGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIR})
link_directories(${OpenGL_LIBRARY_DIR})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set (LIBELAS_SRC_DIR "../src/reconstruct/libelas")
include_directories(${LIBELAS_SRC_DIR})
SET(CMAKE_CXX_FLAGS "-msse3")
FILE (GLOB LIBELAS_SRC_FILES "../src/reconstruct/libelas/*.cpp")

include_directories(${irdv_sim_SOURCE_DIR}/src)

set(CMAKE_BUILD_TYPE Debug)

add_executable(main
../src/main.cpp
../src/basefunc/basefunc.cpp
../src/calib/stereocalib.cpp
../src/rectify/stereorectify.cpp
../src/reconstruct/stereoreconstruction.cpp

../src/reconstruct/libelas/descriptor.cpp
../src/reconstruct/libelas/elas.cpp
../src/reconstruct/libelas/filter.cpp
../src/reconstruct/libelas/matrix.cpp
../src/reconstruct/libelas/triangle.cpp
#${LIBELAS_SRC_FILES}
)

target_link_libraries( main ${OpenCV_LIBRARIES} )
target_link_libraries( main ${OPENGL_LIBRARIES} )
target_link_libraries( main ${GLUT_LIBRARIES} )
target_link_libraries( main ${PCL_LIBRARIES} )

#install (TARGETS main DESTINATION ./bin)











