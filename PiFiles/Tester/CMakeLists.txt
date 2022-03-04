cmake_minimum_required(VERSION 3.17)
project(Tester)

include_directories("C:/mingw64/include")
set(CMAKE_CXX_STANDARD 20)

find_package(nlohmann_json CONFIG REQUIRED)

add_executable(Tester main.cpp TestSensorLib.cpp TestSensorLib.h)

target_link_libraries(Tester PRIVATE nlohmann_json nlohmann_json::nlohmann_json sfml-graphics sfml-system sfml-window sfml-main)