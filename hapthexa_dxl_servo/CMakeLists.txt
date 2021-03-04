cmake_minimum_required(VERSION 3.5)
project(hapthexa_dxl_servo)

#message("start")
#get_cmake_property(vns VARIABLES)
#foreach(vn ${vns})
#	message("${vn}=${${vn}}")
# endforeach()
# message("end")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (APPLE)
    # This is a bug in CMake that causes it to prefer the system version over
    # the one in the specified ROOT folder.
    set(OPENSSL_ROOT_DIR ${OPENSSL_ROOT_DIR} /usr/local/Cellar/openssl@1.1/1.1.1g/)
    set(OPENSSL_CRYPTO_LIBRARY ${OPENSSL_ROOT_DIR}/lib/libcrypto.dylib CACHE FILEPATH "" FORCE)
    set(OPENSSL_SSL_LIBRARY ${OPENSSL_ROOT_DIR}/lib/libssl.dylib CACHE FILEPATH "" FORCE)
endif()
find_package(OpenSSL REQUIRED)

add_compile_options(-Wall -g -DdTRIMESH_ENABLED -DdDOUBLE)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hapthexa_msgs REQUIRED)


# build dynamixel workbench as static library
add_library(dynamixel_workbench STATIC
  dynamixel_workbench_toolbox/dynamixel_item.cpp
  dynamixel_workbench_toolbox/dynamixel_driver.cpp
  dynamixel_workbench_toolbox/dynamixel_tool.cpp
  dynamixel_workbench_toolbox/dynamixel_workbench.cpp
)
if(APPLE)
  target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_mac_cpp.dylib")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
#  message("Linux SBC!")
  target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_sbc_cpp.so")
else()
  target_link_libraries(dynamixel_workbench LINK_PUBLIC "/usr/local/lib/libdxl_x64_cpp.so")
endif()

add_executable(hapthexa_servo src/hapthexa_servo.cpp)
target_include_directories(hapthexa_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hapthexa_servo dynamixel_workbench)
ament_target_dependencies(hapthexa_servo rclcpp hapthexa_msgs)

install(
  TARGETS hapthexa_servo
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB SCRIPT_FILES scripts/*)
install(
  PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

# configフォルダをインストール
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
