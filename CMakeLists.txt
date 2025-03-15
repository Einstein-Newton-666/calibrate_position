cmake_minimum_required(VERSION 3.8)
project(calibrate_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

###########
## Build ##
###########
ament_auto_add_executable(${PROJECT_NAME} DIRECTORY src)

target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
