cmake_minimum_required(VERSION 3.16)
project(navtrack LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(NAVTRACK_SANITIZE "Build with AddressSanitizer and UBSan" OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  set(WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror)
  if(NAVTRACK_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

# C core: no heap allocation, suitable for a microcontroller.
add_library(navcore STATIC c/src/nmea.c c/src/ring_buffer.c c/src/geo.c)
target_include_directories(navcore PUBLIC c/include)
target_compile_options(navcore PRIVATE ${WARNINGS})
target_link_libraries(navcore PUBLIC m)

add_library(navtrack_lib STATIC cpp/src/decoder.cpp cpp/src/kalman.cpp cpp/src/track.cpp cpp/src/simulator.cpp)
target_include_directories(navtrack_lib PUBLIC cpp/include)
target_link_libraries(navtrack_lib PUBLIC navcore)
target_compile_options(navtrack_lib PRIVATE ${WARNINGS})

add_executable(navtrack cpp/src/main.cpp)
target_link_libraries(navtrack PRIVATE navtrack_lib)
target_compile_options(navtrack PRIVATE ${WARNINGS})

enable_testing()
add_executable(test_core tests/test_core.c)
target_link_libraries(test_core PRIVATE navcore)
target_compile_options(test_core PRIVATE ${WARNINGS})
add_test(NAME core COMMAND test_core)

add_executable(test_navtrack tests/test_navtrack.cpp)
target_link_libraries(test_navtrack PRIVATE navtrack_lib)
target_compile_options(test_navtrack PRIVATE ${WARNINGS})
add_test(NAME navtrack COMMAND test_navtrack)
