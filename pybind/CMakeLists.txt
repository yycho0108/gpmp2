include(FetchContent)

set(PYBIND11_PYTHON_VERSION "3.6")

# Extra packages needed internally
find_package(OpenCV REQUIRED)

# pybind11
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY git@github.com:pybind/pybind11.git
  GIT_TAG v2.6.2
  GIT_SHALLOW 1
  SOURCE_DIR
  pybind11)
FetchContent_MakeAvailable(pybind11)

pybind11_add_module(pygpmp2 SHARED module.cpp
    interface.hpp interface.cpp)

target_compile_definitions(pygpmp2 PUBLIC PYTHON_EXECUTABLE="/usr/bin/python3")

# link
target_link_libraries(pygpmp2 PUBLIC gpmp2)
target_link_libraries(pygpmp2 PRIVATE ${OpenCV_LIBS})

# incl
target_include_directories(
  pygpmp2 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../>)
target_include_directories(pygpmp2 PRIVATE ${OpenCV_INCLUDE_DIRS})

set_target_properties(pygpmp2 PROPERTIES EXPORT_NAME pygpmp2)
