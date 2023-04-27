# ROS2 Workspace

in the folder ADS run: 
`meson build`  
`ninja -C build´  

## commands to run the nodes: 
ros2 run ads_example_package ads_node
ros2 run crane_controller controller
ros2 run crane_controller hmi
ros2 launch urdf_tutorial crane_visualization.launch.py

## display the topics in the command line:
ros2 topic echo /state_publisher
ros2 topic echo /motion_reference

don't forget to build and source the solution
