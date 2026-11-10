cmake_minimum_required(VERSION 3.8)
project(epos2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(EPOS_COMMAND_ROOT "$ENV{HOME}/opt/epos-command-library/EPOS_Linux_Library"
  CACHE PATH "Root directory of the extracted Maxon EPOS Linux Library")

string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" EPOS_SYSTEM_PROCESSOR)
if(EPOS_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(EPOS_LIBRARY_ARCH "intel/x86_64")
elseif(EPOS_SYSTEM_PROCESSOR MATCHES "^(x86|i[3-6]86)$")
  set(EPOS_LIBRARY_ARCH "intel/x86")
elseif(EPOS_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(EPOS_LIBRARY_ARCH "arm/v8")
elseif(EPOS_SYSTEM_PROCESSOR MATCHES "^armv7")
  set(EPOS_LIBRARY_ARCH "arm/v7")
elseif(EPOS_SYSTEM_PROCESSOR MATCHES "^armv6")
  set(EPOS_LIBRARY_ARCH "arm/v6")
else()
  message(FATAL_ERROR
    "Unsupported CPU architecture '${CMAKE_SYSTEM_PROCESSOR}' for the Maxon "
    "EPOS Linux Library. Set EPOS_COMMAND_LIBRARY to the shared library "
    "for your system.")
endif()

if(NOT EPOS_COMMAND_INCLUDE_DIR OR EPOS_COMMAND_INCLUDE_DIR MATCHES "-NOTFOUND$")
  set(EPOS_COMMAND_INCLUDE_DIR "${EPOS_COMMAND_ROOT}/include"
    CACHE PATH "Directory containing the Maxon Definitions.h header" FORCE)
endif()
if(NOT EPOS_COMMAND_LIBRARY OR EPOS_COMMAND_LIBRARY MATCHES "-NOTFOUND$")
  set(EPOS_COMMAND_LIBRARY
    "${EPOS_COMMAND_ROOT}/lib/${EPOS_LIBRARY_ARCH}/libEposCmd.so.6.8.1.0"
    CACHE FILEPATH "Path to the Maxon EPOS Command Library shared object" FORCE)
endif()
set(EPOS_FTDI_LIBRARY
  "${EPOS_COMMAND_ROOT}/lib/${EPOS_LIBRARY_ARCH}/libftd2xx.so.1.4.8")
set(EPOS_LIBRARY_DIRECTORY "${EPOS_COMMAND_ROOT}/lib/${EPOS_LIBRARY_ARCH}")
find_library(EPOS_USB_LIBRARY NAMES usb-1.0)

if(NOT EXISTS "${EPOS_COMMAND_INCLUDE_DIR}/Definitions.h")
  message(FATAL_ERROR
    "Maxon header not found: ${EPOS_COMMAND_INCLUDE_DIR}/Definitions.h. "
    "Download and extract the EPOS Linux Library to ${EPOS_COMMAND_ROOT}, "
    "or set EPOS_COMMAND_INCLUDE_DIR.")
endif()
if(NOT EXISTS "${EPOS_COMMAND_LIBRARY}")
  message(FATAL_ERROR
    "Maxon library not found: ${EPOS_COMMAND_LIBRARY}. Download and extract "
    "the EPOS Linux Library to ${EPOS_COMMAND_ROOT}, or set "
    "EPOS_COMMAND_LIBRARY.")
endif()
if(NOT EXISTS "${EPOS_FTDI_LIBRARY}")
  message(FATAL_ERROR "Maxon FTDI library not found: ${EPOS_FTDI_LIBRARY}.")
endif()
if(NOT EPOS_USB_LIBRARY)
  message(FATAL_ERROR
    "libusb-1.0 was not found. Install the development package "
    "(Ubuntu: libusb-1.0-0-dev).")
endif()

function(create_epos_library_link link_name target_name)
  if(NOT EXISTS "${EPOS_LIBRARY_DIRECTORY}/${link_name}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${target_name}" "${EPOS_LIBRARY_DIRECTORY}/${link_name}"
      RESULT_VARIABLE EPOS_LINK_RESULT
    )
    if(NOT EPOS_LINK_RESULT EQUAL 0)
      message(FATAL_ERROR
        "Could not create ${EPOS_LIBRARY_DIRECTORY}/${link_name}. "
        "Check write permissions for the extracted Maxon library directory.")
    endif()
  endif()
endfunction()

create_epos_library_link(libEposCmd.so libEposCmd.so.6.8.1.0)
create_epos_library_link(libftd2xx.so libftd2xx.so.1.4.8)

add_executable(epos_driver_node src/epos_driver_node.cpp)
target_compile_features(epos_driver_node PUBLIC cxx_std_17)
target_include_directories(epos_driver_node PRIVATE ${EPOS_COMMAND_INCLUDE_DIR})
ament_target_dependencies(epos_driver_node rclcpp std_msgs std_srvs)
target_link_libraries(epos_driver_node
  ${EPOS_COMMAND_LIBRARY}
  ${EPOS_FTDI_LIBRARY}
  ${EPOS_USB_LIBRARY}
)

install(TARGETS epos_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_library(epos2_test_fake SHARED test/fake_epos.cpp)
  target_include_directories(epos2_test_fake PRIVATE ${EPOS_COMMAND_INCLUDE_DIR})
  add_test(
    NAME epos_driver_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_network.py
      $<TARGET_FILE:epos_driver_node> $<TARGET_FILE:epos2_test_fake>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml ${EPOS_LIBRARY_DIRECTORY}
  )
  set_tests_properties(epos_driver_integration PROPERTIES TIMEOUT 90)
endif()

ament_package()
