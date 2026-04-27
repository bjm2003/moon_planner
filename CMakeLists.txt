cmake_minimum_required(VERSION 3.16)

project(moon_planner VERSION 0.1.0 LANGUAGES CXX)

include(cmake/ProjectOptions.cmake)
include(cmake/Sanitizers.cmake)

add_library(moon_planner_core
  src/core/geometry.cpp
  src/config/planner_config.cpp
  src/config/vehicle_config.cpp
  src/config/cost_config.cpp
  src/model/skid_steer_model.cpp
  src/model/motion_constraints.cpp
  src/primitives/primitive_library.cpp
  src/primitives/primitive_generator.cpp
  src/map/grid_index.cpp
  src/map/occupancy_grid.cpp
  src/map/elevation_grid.cpp
  src/map/cost_map.cpp
  src/map/history_layer.cpp
  src/map/map_fusion.cpp
  src/collision/footprint.cpp
  src/collision/collision_checker.cpp
  src/cost/traversability_evaluator.cpp
  src/cost/primitive_cost.cpp
  src/cost/heuristic.cpp
  src/search/open_list.cpp
  src/search/state_lattice.cpp
  src/search/hybrid_astar.cpp
  src/planner/lattice_planner.cpp
  src/planner/local_planner.cpp
  src/planner/recovery_planner.cpp
  src/trajectory/path_extractor.cpp
  src/trajectory/trajectory_generator.cpp
  src/trajectory/trajectory_smoother.cpp
  src/runtime/memory_pool.cpp
  src/runtime/planner_context.cpp
  src/runtime/scheduler.cpp
  src/runtime/diagnostics.cpp
  src/io/yaml_loader.cpp
  src/io/map_reader.cpp
  src/io/map_writer.cpp
  src/io/primitive_io.cpp
  src/io/scenario_reader.cpp
  src/io/trajectory_writer.cpp
)

target_include_directories(moon_planner_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(moon_planner_core PUBLIC cxx_std_17)
moon_planner_apply_project_options(moon_planner_core)

add_executable(moon_planner_cli src/apps/moon_planner_cli.cpp)
target_link_libraries(moon_planner_cli PRIVATE moon_planner_core)
moon_planner_apply_project_options(moon_planner_cli)

add_executable(generate_primitives src/apps/generate_primitives.cpp)
target_link_libraries(generate_primitives PRIVATE moon_planner_core)
moon_planner_apply_project_options(generate_primitives)

add_executable(benchmark_planner src/apps/benchmark_planner.cpp)
target_link_libraries(benchmark_planner PRIVATE moon_planner_core)
moon_planner_apply_project_options(benchmark_planner)

enable_testing()

add_executable(test_grid_index tests/unit/test_grid_index.cpp)
target_link_libraries(test_grid_index PRIVATE moon_planner_core)
add_test(NAME test_grid_index COMMAND test_grid_index)

add_executable(test_skid_steer_model tests/unit/test_skid_steer_model.cpp)
target_link_libraries(test_skid_steer_model PRIVATE moon_planner_core)
add_test(NAME test_skid_steer_model COMMAND test_skid_steer_model)

add_executable(test_collision_checker tests/unit/test_collision_checker.cpp)
target_link_libraries(test_collision_checker PRIVATE moon_planner_core)
add_test(NAME test_collision_checker COMMAND test_collision_checker)

add_executable(test_cost_map tests/unit/test_cost_map.cpp)
target_link_libraries(test_cost_map PRIVATE moon_planner_core)
add_test(NAME test_cost_map COMMAND test_cost_map)

add_executable(test_local_planner tests/unit/test_local_planner.cpp)
target_link_libraries(test_local_planner PRIVATE moon_planner_core)
add_test(NAME test_local_planner COMMAND test_local_planner)

add_executable(test_scenario_reader tests/unit/test_scenario_reader.cpp)
target_link_libraries(test_scenario_reader PRIVATE moon_planner_core)
add_test(NAME test_scenario_reader COMMAND test_scenario_reader)
set_tests_properties(test_scenario_reader PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_planner_flat_map tests/integration/test_planner_flat_map.cpp)
target_link_libraries(test_planner_flat_map PRIVATE moon_planner_core)
add_test(NAME test_planner_flat_map COMMAND test_planner_flat_map)

add_executable(test_planner_dense_obstacles tests/integration/test_planner_dense_obstacles.cpp)
target_link_libraries(test_planner_dense_obstacles PRIVATE moon_planner_core)
add_test(NAME test_planner_dense_obstacles COMMAND test_planner_dense_obstacles)
set_tests_properties(test_planner_dense_obstacles PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(test_planner_slope_map tests/integration/test_planner_slope_map.cpp)
target_link_libraries(test_planner_slope_map PRIVATE moon_planner_core)
add_test(NAME test_planner_slope_map COMMAND test_planner_slope_map)
set_tests_properties(test_planner_slope_map PROPERTIES WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
