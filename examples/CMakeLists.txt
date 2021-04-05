cmake_minimum_required(VERSION 2.6)
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(fmt REQUIRED)


add_executable(example-kaist main.cpp)
target_link_libraries(example-kaist PUBLIC ${PROJECT_NAME})
target_link_libraries(example-kaist PRIVATE ${OpenCV_LIBS})
target_link_libraries(example-kaist PRIVATE Boost::program_options)
target_link_libraries(example-kaist PRIVATE fmt)

target_include_directories(example-kaist PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>)
target_include_directories(example-kaist PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(example-kaist PRIVATE ${Boost_INCLUDE_DIRS})
