include_directories("${CMAKE_SOURCE_DIR}/include")
include_directories(tensorflow)

find_package(pybind11 CONFIG)
file(GLOB_RECURSE THE_SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/*.cc)
list(FILTER THE_SOURCE_FILES EXCLUDE REGEX ".*(space_sqfd.cc|dummy_app.cc|main.cc)")
pybind11_add_module(nmslib nmslib.cc tensorflow/cpu_feature_guard.cc tensorflow/cpu_info.cc ${THE_SOURCE_FILES})
