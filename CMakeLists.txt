cmake_minimum_required(VERSION 3.1)

project(mc_nao)
set(CMAKE_CXX_STANDARD 17)

find_package(mc_rtc REQUIRED)

find_description_package(nao_description)

message(STATUS "NAO_DESCRIPTION_PATH: ${NAO_DESCRIPTION_PATH}")
configure_file(include/mc_nao/config.h.cmake.in
               "${CMAKE_CURRENT_BINARY_DIR}/include/mc_nao/config.h")

add_robot(nao src/nao.cpp include/mc_nao/nao.h)
target_include_directories(nao PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
target_include_directories(nao PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

install(FILES etc/LIPMWalking/nao.yaml
        DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/LIPMWalking/)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
