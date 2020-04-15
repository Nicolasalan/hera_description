# hera_description

This package contains the URDF Description model and simulation of the [HERA robot (2020 version)](http://robofei.aquinno.com/athome/wp-content/uploads/2020/01/TDP2020ROBOFEI.pdf), developed by the [RoboFEI@home Team](http://robofei.aquinno.com/athome/) in the [FEI University Center](https://portal.fei.edu.br/).

![HERA](/doc/HERA.png)

## Dependencies:
* [ROS](https://www.ros.org/) (Melodic Morenia)
  * [roslaunch](http://wiki.ros.org/roslaunch)
  * [rviz](http://wiki.ros.org/rviz)
* [Gazebo](http://gazebosim.org/) (9.0.0)


## File structure:
```
|-- hera_description
  |-- config
  |-- doc
  |-- launch
  |-- meshes
  |-- robots
  |-- urdf
  |-- CMakeLists.txt
  |-- package.xml
  |-- README.md
  |-- todo.md
```

### config:
This folder contains parameters configurations used to visualize and control the robot.
```
  |-- config
    |-- rviz
      |-- vizualize_model.rviz
    |-- yaml
      |-- hera_control.yaml
```

### doc
This folder contains files used in this markdown document.
```
  |-- doc
    .
    .
    .
```

### launch:
This folder contains files used to launch the robot using roslaunch in the ROS ecosystem.
```
  |-- launch
    |-- load_description.launch
    |-- vizualize_model.launch
```
* **load_description**: Used to load the robot model in ROS and Gazebo. Parameters:
  * **robot_model**: (string, default: hera_full) - Robot used. Available into 'robots' folder.
  * **robot_name**: (string, default: robot) - Robot name used as reference in simulation.
  * **init_pos_x**: (double, default: 0.0) - Position x of the robot in simulation.
  * **init_pos_y**: (double, default: 0.0) - Position y of the robot in simulation.
  * **init_pos_z**: (double, default: 0.0) - Position z of the robot in simulation.
  * **init_yaw**: (double, default: 0.0) - yaw of the robot in simulation.
  * **use_jsp_gui**: (boolean, default: false) - Start joint_state_publisher gui
* **vizualize_model**: Used to visualize the robot. Init gazebo, rviz and call **load_description**. Parameters:
  * **robot_model**: (string, default: hera_full) - Same as **load_description**.
  * **enable_rviz**: (bool, default:true) - Start rviz.
  * **enable_gazebo**: (bool, default:true) - Start gazebo.

### meshes:
All meshes are used only to composite the robot visualization. The robot parts were modeled by
[William Yaguiu](mailto:williamyaguiu@gmail.com) with [Solidworks 2018](https://www.solidworks.com/) and [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) plugin and the sensors were exported from [GrabCAD Community](https://grabcad.com/).
```
  |-- meshes
    |--asus_xtion.stl
    |--base.stl
    |--head.stl
    |--hokuyo_urg.dae
    |--hokuyo_utm.stl
    |--kinect_one.stl
    |--torso.stl
    |--torso_sensor_plat.stl
    |--wheelB.stl
    |--wheelF.stl

```
<img src="doc/asus_xtion.png" width="150">
<img src="doc/base.png" width="150">
<br/>
<img src="doc/head.png" width="150">
<img src="doc/hokuyo_urg.png" width="150">
<br/>
<img src="doc/hokuyo_utm.png" width="150">
<img src="doc/kinect_one.png" width="150">
<br/>
<img src="doc/torso.png" width="150">
<img src="doc/torso_sensor_plat.png" width="150">
<br/>
<img src="doc/wheelB.png" width="150">
<img src="doc/wheelF.png" width="150">

### Robots:
Different versions of robots available.
```
  |-- robots
    |--hera_1.urdf.xacro
    |--hera_2.urdf.xacro
    |--hera_3.urdf.xacro
    |--hera_4.urdf.xacro
    |--hera_full.urdf.xacro
```
* **hera_1**: base (**OK**)
  * Links and Joints: [hera_1](doc/hera_1.pdf).
  * Collision mode (left) and visual mode (right). <br/>
<img src="doc/1-1-collision.png" width="150">
<img src="doc/1-1-visual.png" width="150">
<br/>
<img src="doc/1-2-collision.png" width="150">
<img src="doc/1-2-visual.png" width="150">
<br/><br/>

* **hera_2**: base + torso (**OK**)
  * Links and Joints: [hera_2](doc/hera_2.pdf).
  * Collision mode (left) and visual mode (right). <br/>
<img src="doc/2-1-collision.png" width="150">
<img src="doc/2-1-visual.png" width="150">
<br/>
<img src="doc/2-2-collision.png" width="150">
<img src="doc/2-2-visual.png" width="150">
<br/><br/>

* **hera_3**: base + torso + head (**OK**)
  * Links and Joints: [hera_3](doc/hera_3.pdf).
  * Collision mode (left) and visual mode (right).  <br/>
<img src="doc/3-1-collision.png" width="150">
<img src="doc/3-1-visual.png" width="150">
<br/>
<img src="doc/3-2-collision.png" width="150">
<img src="doc/3-2-visual.png" width="150">
<br/><br/>

* **hera_4**: base + torso + manipulator (**Not done - same as hera_3**)
<br/><br/>

* **hera_full**: base + torso + head + manipulator (**Not done - same as hera_3**)

### urdf
All urdf code used to model the robot.
```
  |-- urdf
    |--actuators
      |--base.urdf.xacro
      |--head.urdf.xacro
      |--manipulator.urdf.xacro
      |--torso.urdf.xacro
    |--sensors
      |--asus_xtion.urdf.xacro
      |--hokuyo_urg.urdf.xacro
      |--hokuyo_utm.urdf.xacro
      |--kinect_one.urdf.xacro
    |--simulation
      |--control.gazebo.xacro
      |--base.gazebo.xacro
      |--asus_xtion.gazebo.xacro
      |--hokuyo_urg.gazebo.xacro
      |--hokuyo_utm.gazebo.xacro
      |--kinect_one.gazebo.xacro
    |--commons.urdf.xacro
```

* **actuators**: Code used in actuators.
* **sensors**: Code used in sensors.
* **simulation**: Gazebo plugins used for simulation.
* **commons**: Colors definitions and macros used in all files.

---

#### Tools:
* **xacro**: Used to convert xacro file to urdf file. ex:
```
  $ rosrun xacro xacro `rospack find hera_description`/robots/hera_full.urdf.xacro > hera.urdf
```
* **check_urdf**: Used to check the urdf consistency. ex:
```
  $ check_urdf hera.urdf
```
* **urdf_to_graphiz**: Used to generate a pdf with robot links and joints. ex:
```
  $ urdf_to_graphiz hera.urdf
```
