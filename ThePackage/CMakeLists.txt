cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(ThePackage)

set(PACKAGE_NAME ThePackage)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)
#target_link_libraries(minimal PRIVATE matplotlib_cpp)
#set_target_properties(minimal PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

find_package(PCL 1.10 REQUIRED)
FIND_PACKAGE( Boost 1.71 REQUIRED COMPONENTS thread filesystem)
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} include)
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} bin)
add_definitions(${PCL_DEFINITIONS})
add_executable(thePackage src/main.cpp)
target_link_libraries(thePackage ${PCL_LIBRARIES} ${Boost_LIBRARIES} libroyale.so Eigen3::Eigen matplotlib_cpp) 