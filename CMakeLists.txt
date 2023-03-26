cmake_minimum_required(VERSION 3.0.0)
project(gpt3_test VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

find_package(PythonLibs REQUIRED)
find_package(PythonInterp REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(include)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})



add_executable(gpt3_test 
               src/main.cpp
               src/joint_imitator.cpp)


target_link_libraries(gpt3_test ${PYTHON_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)



enable_testing()

add_executable(
  joint_position_calculation_test
  test/joint_position_calculation/joint_position_calculation.cc
  src/joint_imitator.cpp
)

target_link_libraries(joint_position_calculation_test ${GTEST_LIBRARIES} pthread)

include(GoogleTest)
gtest_discover_tests(joint_position_calculation_test )