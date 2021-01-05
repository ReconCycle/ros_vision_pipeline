# for details, see: http://www.artificialhumancompanions.com/structure-python-based-ros-package/
# code also here: https://github.com/SimonBirrell/simontest

cmake_minimum_required(VERSION 2.8.3)
project(vision-pipeline)

# find_package(catkin REQUIRED COMPONENTS
#   rospy
#   std_msgs
# )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)