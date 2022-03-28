cmake_minimum_required(VERSION 3.1)

project(mc_nao)
set(CMAKE_CXX_STANDARD 11)

find_package(nao_description REQUIRED)
find_package(mc_rtc REQUIRED)

if("${nao_description_INSTALL_PREFIX}" STREQUAL "")
  if("${nao_description_SOURCE_PREFIX}" STREQUAL "")
    message(FATAL_ERROR "Your nao_description package does not define where to find the data")
  endif()
  set(NAO_DESCRIPTION_PATH "${nao_description_SOURCE_PREFIX}")
else()
  set(NAO_DESCRIPTION_PATH "${nao_description_INSTALL_PREFIX}")
endif()

message(STATUS "NAO_DESCRIPTION_PATH: ${NAO_DESCRIPTION_PATH}")
configure_file(include/mc_nao/config.h.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/include/mc_nao/config.h")

add_robot(nao src/nao.cpp include/mc_nao/nao.h)
target_include_directories(nao PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(nao PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
