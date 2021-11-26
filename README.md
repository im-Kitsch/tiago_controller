# tiago_controller

Tiago controller based on inria_wbc.

## References:
- manual for Tiago: https://gitlab.inria.fr/larsen-robots/tiago/-/tree/master/doc
- controller by Brice: https://gitlab.inria.fr/locolearn/tiago_controller/-/tree/master 
- controller for Talos: https://gitlab.inria.fr/locolearn/talos_controller 

## Usage (notes)

`./run_docker.sh -it --name tiago registry.gitlab.inria.fr/locolearn/docker_talos/inria_wbc_pal:tiago -c terminator` 

### compilation
- `source devel/setup.bash` 
- `catkin_make --only-pkg-with-deps tiago_controller` 

### Gazebo
- `source catkin_ws/devel/setup.bash`
- `roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel` 

### controller
- rqt -> controller manager -> stop torso/head/gripper/arm controllers
- `source catkin_ws/devel/setup.bash` 
- `roslaunch tiago_controller tiago_controller.launch`


### services
- `rosservice call /tiago_controller/move  "{pose: {position: {x: 0.5, y: 0.5, z: 1.2}}, duration: 1., use_orientation: False, use_position: True, task_name: ee}" 

