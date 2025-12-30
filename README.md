## 🔧 About the Repo 
This repo contains the package for simulating differential drive robot using **ROS2 (Robot Operating System 2)**  and **Gazebo** simulator. 

 **ROS2 Distro Used :** Jazzy Jalisco 
 
 **Gazebo Version:** Gazebo Harmonic 
 
 **Ubuntu Version Required:** Ubuntu 24.04 
 
### What’s ROS2?
The Robot Operating System 2 (ROS 2) is an open-source middleware framework designed to simplify the development of complex robotic systems. Despite the name, it is not a traditional operating system but a collection of software libraries and tools that enable different parts of a robot (nodes) to communicate reliably in real-time. By providing standardized communication protocols like Topics, Services, and Actions, ROS 2 allows developers to build modular applications where sensors, controllers, and AI algorithms can interact seamlessly across different hardware platforms.

For more information check out the website:
👉 [ROS: Home](https://www.ros.org/)

 ### Official Documentation
For the most up-to-date information on the latest Long-Term Support (LTS) release, visit the official documentation:

👉 [ROS 2 Jazzy Jalisco Documentation](https://docs.ros.org/en/jazzy/index.html)

## 📦 Package overview
The name of the package is diffdrive_description. Given below is the package structure. This package uses **ros2_control** which has built in **diff_drive_controller**  for controlling **differential drive robot** instead of the gazebo differential drive plugin
<br/>
```text
diffdrive_description
    ├── config
    │   ├── my_robot_controllers.yaml
    │   └── ros_gz_bridge_gazebo.yaml
    ├── diffdrive_description
    │   └── __init__.py
    ├── launch
    │   ├── display.launch.py
    │   └── gazebo.launch.xml
    ├── meshes
    │   └── cam1.stl
    ├── package.xml
    ├── resource
    │   └── diffdrive_description
    ├── setup.cfg
    ├── setup.py
    ├── test
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── urdf
    │   ├── diff_drive_macro.xacro
    │   ├── diff_drive.ros2control
    │   ├── diff_drive.xacro
    │   └── materials.xacro
    └── worlds
        └── obstacles.sdf
```

<br/>

**Config :** Consist files for adjusting the parameters related to controllers and bridge file which acts as bridge between ROS2 and Gazebo

**launch:** Consists of files need for launching Differential drive bot in gazebo for simulation and also spawns the controllers (gazebo.launch.xml)and rviz2 which is used primarily for visualization of data and examination of joints using joint state publisher gui (display.launch.py)

**urdf:** URDF stands for Unified Robot Description Format . Consist of files which describe how the robot will look , its joints and inertia

**meshes:** Consists of stl files which are used in URDF . Stl file given here was designed using Fusion 360 .

**worlds:** Consists files for running custom worlds in gazebo
 
The rest of the directories are created by default when the package was created
which are necessary for managing dependencies and intialising directories.




## 🚀 Launching Simulation 

Before launching it is necessary to clone this package into particular workspace and build the package using the command

```text
colcon build --packages-select diffdrive_description
```
yu can also go for building multiple packages by including the name of the package after diffdrive_description or if u want to  build all the packages in the workspace ,run the command

```text
colcon build
```


For Launching the simulation open the Ubuntu terminal and run the command
```text

ros2 launch diffdrive_description gazebo.launch.xml
```

This launches the gazebo simulator with differential drive bot spawned inside the custom world that is included in the worlds directory obstacles.sdf


##  ⚙️ Driving the bot

For driving the bot in the given simulated custom world , open the ubuntu terminal and run the command
```text

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```




[Differential_drive_robot.webm](https://github.com/user-attachments/assets/c84d1fda-0aeb-41a2-9b0d-619b86d65b51)















