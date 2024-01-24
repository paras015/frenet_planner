cmake_minimum_required(VERSION 3.10)
project(frenet_planner)

SET (PYTHON_PATH "/usr/include/python2.7/")
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)

add_executable(frenet_planner src/frenet_planner.cpp src/cubic_spline.cpp src/polynomials.cpp)
target_include_directories(frenet_planner PRIVATE include)
target_link_libraries(frenet_planner 
                      Eigen3::Eigen
                      ${EIGEN3_LIBRARIES}
                      ${PYTHON_LIBRARIES})
target_include_directories(frenet_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(frenet_planner ${PYTHON_LIBRARIES})