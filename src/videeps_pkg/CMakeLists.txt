cmake_minimum_required(VERSION 3.8)
project(videeps_pkg)


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/SetTargetObject.srv"
)
ament_package()
