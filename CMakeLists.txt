cmake_minimum_required(VERSION 3.8)
project(mpc_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify include directories
include_directories(
  include/mpc_controller_pkg/cpp_include  # C++ headers
  include/mpc_controller_pkg/c_include    # C headers
)

# Collect all C source files and their corresponding object files
set(C_SOURCES
  src/c_src/euler1.c
  src/c_src/eulermod2.c
  src/c_src/heun2.c
  src/c_src/ruku45.c
  src/c_src/rodas.c
  src/c_src/trapezodial.c
  src/c_src/simpson.c
  src/c_src/grampc_alloc.c
  src/c_src/grampc_fixedsize.c
  src/c_src/grampc_init.c
  src/c_src/grampc_mess.c
  src/c_src/grampc_run.c
  src/c_src/grampc_setopt.c
  src/c_src/grampc_setparam.c
  src/c_src/grampc_util.c
  src/c_src/tracking.c
)

# Collect all C++ source files
set(CPP_SOURCES
  src/cpp_src/grampc.cpp
  src/cpp_src/problem_description.cpp
  src/cpp_src/UAV_model.cpp
)

# Create the static library 'grampc' from both C and C++ sources
add_library(grampc STATIC ${C_SOURCES} ${CPP_SOURCES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(mpc_controller src/mpc_controller.cpp)
add_executable(sim_pix src/sim_pix.cpp)

# Link the executables with the static library
target_link_libraries(mpc_controller grampc)
target_link_libraries(sim_pix grampc)

ament_target_dependencies(mpc_controller rclcpp px4_msgs)
ament_target_dependencies(sim_pix rclcpp px4_msgs) 

install(TARGETS
  mpc_controller
  sim_pix
  grampc
  DESTINATION lib/${PROJECT_NAME})

ament_package()
