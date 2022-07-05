cmake_minimum_required(VERSION 3.1.0)
project(serial_linux)

add_library(
    serial
    src/serial/Serial.cpp
)

target_include_directories(serial
  PRIVATE
    include/
)

add_executable(
    main
    src/main.cpp
)
target_include_directories(main
  PRIVATE
    include/
)
target_link_libraries(main serial)