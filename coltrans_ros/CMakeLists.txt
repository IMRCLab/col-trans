cmake_minimum_required(VERSION 3.8)
project(coltrans_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  coltrans_ros/downwash.py
  coltrans_ros/figure8.py
  coltrans_ros/helix.py
  coltrans_ros/hello_world.py
  coltrans_ros/payload_figure8.py
  coltrans_ros/payload_test.py
  coltrans_ros/testyaw.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  config
  data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
