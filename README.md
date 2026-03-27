# ROS BOT

## Modules required
  - cvzone
  - ultralytics
  - numpy<2 `pip install "numpy<2"`

## Steps to run
  - `colcon build`
  - `source install/local_setup.sh`
  - Run 6 terminals and source in all of them
    - `ros2 launch bme_ros2_navigation spawn_robot.launch.py`
  - Gazebo will open add a standing person and a firehydrant(object of your choice just check wether it exist in out yolo model) (remove the already existing fire hydrant in the world)
  - In an other terminal run
    - `ros2 launch bme_ros2_navigation navigation_with_slam.launch.py`
  - In an other terminal run 
    - `ros2 launch object_finder full_launch.py`
  - In another terminal run
    - `ros2 run bme_gazebo_sensors_py set_target_client`
  - After running prompt the object you want to pick up and give to the person(fire hydrant in this case)
  - In another terminal run
    - `ros2 run bme_ros2_simple_arm_py object_pickup`
  - In another terminal run
    - `ros2 run bme_ros2_simple_arm_py object_detection`
   
  - If you want to see the yolo camera object detection in action open rviz add display and show the /detection/image topic  
