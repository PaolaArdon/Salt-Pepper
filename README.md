# Salt-Pepper Project 
This is the Robotics Project (VIBOT 3rd semester, Heriot-Watt University) done by Paola Ardon(ardonp@hotmail.com) Kaisar Kushibar(k.kushibar@gmail.com), and Songyou Peng (psy920710@gmail.com). 

### Requirements
* [ROS](http://www.ros.org/)
* [OpenCV]()
* [ros-naoqi](https://github.com/ros-naoqi/pepper_robot)
* [ros-teleop](https://github.com/ros-teleop/teleop_tools)
* [ORB SLAM 2](https://github.com/raulmur/ORB_SLAM2)


### Demo
The demonstration of SLAM + Object recognition with Pepper robot can be found [**here**](https://www.youtube.com/watch?v=evFsnWH_bpY&t=5s).


### How to run
First of all, start ROS and connect the host computer with Pepper:
```sh
$ roscore
$ roslaunch pepper_bringup pepper_full_py.launch nao_ip:="ROBOT_IP" roscore_ip:="HOST_IP"
```

Run JoyPepper with Autonomous life ON. This will allow to control the robot using the joystick, but Pepper behaves like a kid (if she gets distracted, she just stops listening you)
```sh
$ rosrun joy joy_node
$ rosrun joy_pepper joypepper.py
```

If you want to control Pepper with Joystick with Autonomous life OFF, add this line before:
```sh
$ roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:="10.42.0.76" network_interface:=wlan0
```
#### Joystick configurations
* Base control: <br />
  * left arrow keys
    * <- turn left, -> right
    * ^ move forward, v move backward
  
* Head control: 
  * right buttons

* Sleep & WakeUp:
  * R1 & R2 buttons


Run ORB SLAM 2 - Monocular
```sh
$ rosrun ORB_SLAM2 Mono /Path/To/ORB_SLAM2/Vocabulary/ORBvoc.bin /Path/To/ORB_SLAM2/Examples/Monocular/TUM1.yaml
```

RUN ORB SLAM - RGB-D
```sh
$ rosrun ORB_SLAM2 RGBD /Path/To/ORB_SLAM2/Vocabulary/ORBvoc.bin /Path/To/ORB_SLAM2/Examples/RGB-D/pepperCameraSettings.yaml false (create a new map) / true (use a saved map)
```

Run object recognition
```sh
$ rosrun pepper_recog recog.py
```

### Logbook
[link](https://www.overleaf.com/6504894skysnw)
