cmake_minimum_required(VERSION 3.5)
project(my_ax12_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  dynamixel_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add other install targets and commands as needed

# Export dependencies
ament_package()
