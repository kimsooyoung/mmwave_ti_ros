cmake_minimum_required(VERSION 3.0)

project(pthread_examples)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER g++)
# set(SOURCES src/main.cpp )

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)


#set(CMAKE_BUILD_TYPE Release)
message("build type: " ${CMAKE_BUILD_TYPE})

add_executable(example1 src/ex1.cpp)
target_link_libraries(example1 pthread)

add_executable(example2 src/ex2.cpp)
target_link_libraries(example2 pthread)

add_executable(example3 src/ex3.cpp)
target_link_libraries(example3 pthread)

add_executable(example4 src/ex4.cpp)
target_link_libraries(example4 pthread)

add_executable(example5 src/ex5.cpp)
target_link_libraries(example5 pthread)

add_executable(example6 src/ex6.cpp)
target_link_libraries(example6 pthread)

include(CPack)