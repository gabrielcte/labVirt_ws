# CubeSat


## Configura labVirt_ws

# Crie o diretorio
mkdir -p dev_ws/src
# Entre no diretorio
cd dev_ws
# Utilize o Colcon para construir seu Workspace
colcon build --symlink-install
# A flag symlink e usada por padrao
ls
#Verifice se as pastas build install log e src foram criadas, se sim, voce tem agora um workspace vazio.
# Os arquivos em ROS sempre sao organizados em packages, e eles devem estar no diretorio src
cd src
#Crie o package
ros2 pkg create --build-type ament_cmake my_package
# Crie a pasta Description Launcher e World
cd
mkdir -p dev_ws/src/labVirt_ws/description
mkdir -p dev_ws/src/labVirt_ws/launch
mkdir -p dev_ws/src/labVirt_ws/worlds

This repo contains an example of a URDF file of a CubeSat and a launch script to run it. (ROS 2)
This is part of the Master's Thesis presented to the Postgraduate Program in Engineering Mechanics of UFABC with the name: "Virtual Laboratory Development: A
Environment for Teaching Modeling and CubeSat Control"

## How To Run


1. Build the package with colcon. `colcon build --symlink-install` and `source install/setup.bash`
2. Launch the `robot_state_publisher` launch file with `ros2 launch my_package rsp_sim.launch.py`.
3. Launch `joint_state_publisher_gui` with `ros2 run joint_state_publisher_gui joint_state_publisher_gui`. You may need to install it if you don't have it already.
4. Launch RViz with `rviz2`
5. Launch to se topics 'ros2 topic echo /imu_plugin/out'


To replicate the RViz display shown in the video you will want to
- Set your fixed frame to `world`
- Add a `RobotModel` display, with the topic set to `/robot_description`, and alpha set to 0.8
- Add a `TF` display with names enabled.
