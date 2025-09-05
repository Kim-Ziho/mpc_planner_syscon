cmake_minimum_required(VERSION 3.8)
project(mpc_planner_solver)

add_compile_options(-std=c++17)

# ACADOS settings
add_definitions(-DACADOS_SOLVER)
string(REPLACE ":" ";" LIBRARY_DIRS $ENV{LD_LIBRARY_PATH})
find_library(acados_LIBRARY libacados.so PATHS ${LIBRARY_DIRS})
find_library(blasfeo_LIBRARY libblasfeo.so PATHS ${LIBRARY_DIRS})
find_library(hpipm_LIBRARY libhpipm.so PATHS ${LIBRARY_DIRS})
# Get the acados path from the LD_LIBRARY_PATH
get_filename_component(acados_path ${acados_LIBRARY} DIRECTORY)
set(acados_path ${acados_path}/..)
set(acados_include_path ${acados_path}/include)
# Print acados_include_path
set(solver_LIBRARIES
    ${PROJECT_SOURCE_DIR}/acados/Solver/libacados_ocp_solver_Solver.so # Generated files
    ${acados_LIBRARY}
    ${blasfeo_LIBRARY}
    ${hpipm_LIBRARY}
)

set(solver_INCLUDE_DIRS
    acados/Solver # Generated files
    acados # Generated files
    ${acados_include_path}
    ${acados_include_path}/blasfeo/include
    ${acados_include_path}/hpipm/include
)
set(solver_SOURCES
    src/acados_solver_interface.cpp
)
# ACADOS settings end

set(DEPENDENCIES
  mpc_planner_util
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${DEPENDENCIES}
  INCLUDE_DIRS
  include
  include/${PROJECT_NAME}
  ${solver_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME}
  ${solver_LIBRARIES}
)

include_directories(
  include
  ${solver_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/mpc_planner_parameters.cpp
  src/state.cpp
  ${solver_SOURCES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${solver_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ${solver_INCLUDE_DIRS}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../
  PATTERN ".svn" EXCLUDE
)
