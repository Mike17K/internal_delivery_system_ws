cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# lib/<pkg> is what `ros2 run` and launch_ros's executable= look in, so the
# teleop UI installs as a PROGRAM there rather than into share/.
install(PROGRAMS scripts/teleop_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
