cmake_minimum_required(VERSION 3.2)
project(dwm1001_dev)

# Set C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

# Build
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(dwm1001_dev_lib SHARED
  src/dwm1001.cpp
  src/kalman_filter.cpp
)

target_link_libraries(dwm1001_dev_lib Eigen3::Eigen)

add_executable(${PROJECT_NAME}
  example/dwm1001_example.cpp
)

target_link_libraries(${PROJECT_NAME} 
  dwm1001_dev_lib
)

# Install.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

INSTALL (DIRECTORY include/
  DESTINATION /usr/local/include
  FILES_MATCHING PATTERN "*.hpp*"
)
