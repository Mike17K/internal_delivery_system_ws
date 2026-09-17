cmake_minimum_required(VERSION 3.8)
project(workcell_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

# Open-RMF nav graph: maps/office_world.building.yaml (hand-authored, see
# scripts/generator/rmf_building_map.py) -> maps/nav_graphs/0.yaml, the
# runtime graph agv_fleet_adapter actually loads. Same pattern as
# rmf_demos_maps' own CMakeLists.txt, minus its gazebo-world/crowd-sim
# steps - we already have our own generated Gazebo world.
set(rmf_building_map "${CMAKE_CURRENT_SOURCE_DIR}/maps/office_world.building.yaml")
set(rmf_nav_graphs_dir "${CMAKE_CURRENT_BINARY_DIR}/maps/nav_graphs")
add_custom_command(
  OUTPUT "${rmf_nav_graphs_dir}/0.yaml"
  COMMAND ros2 run rmf_building_map_tools building_map_generator nav
          "${rmf_building_map}" "${rmf_nav_graphs_dir}"
  DEPENDS "${rmf_building_map}"
)
add_custom_target(generate_office_world_nav_graph ALL
  DEPENDS "${rmf_nav_graphs_dir}/0.yaml"
)
install(DIRECTORY "${rmf_nav_graphs_dir}" DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
