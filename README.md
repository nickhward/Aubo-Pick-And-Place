# Aubo-Pick-And-Place

A way to perform pick and place operations using deep learning algorithms GPD and YOLO. 


## Required Installations

The links to where one can install each program are provided.

- Ubuntu 16.04
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Python 2.7
- [Darknet_ros](https://github.com/leggedrobotics/darknet_ros) -V: YOLOv4
- [OpenCV](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/) >= 3.4
- [GPD](https://github.com/atenpas/gpd)
- [gpd_ros](https://github.com/atenpas/gpd_ros)
- [Moveit](https://moveit.ros.org/install/)
- [PCL](https://www.programmersought.com/article/52981999118/) >= 1.7
- [Aruco_Ros](https://github.com/pal-robotics/aruco_ros)
- [Aubo_I5_Driver](https://github.com/AuboRobot/aubo_robot)
- [Aubo I5 Model](https://github.com/hai-h-nguyen/aubo-i5-full)
## Hardware
- Asus Xtion Pro RGB-D Camera
- Aubo I5 Robot
- Robotiq 85 Gripper
## Running the Programs to Pick Up Object

1. Run the camera calibrated launch files in seperate terminals

```
   roslaunch jsk_pcl_ros openni2_local.launch
   roslaunch jsk_pcl_ros openni2_remote.launch
```

2. In a new terminal run object detection through darknet_ros by command:

```
   roslaunch darknet_ros darknet_ros.launch
```

3. In a new terminal run the program to filter out all point cloud data except the object that was chosen:

```
   rosrun my_aubo_i5_robot_config pass_through
```

4. To find the transformation of the Aubo Robot base link with respect to the camera frame run:

```
   roslaunch static_frame.launch
```

5. Launch the Aubo Driver in a new terminal:

```
   roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=your-robot-ip 
```

6. Grasp the object by running in a new terminal:

```
   rosrun my_aubo_i5_robot_config pick_place
```



## Inner Workings of the Algorithm
**Camera Calibration and Running Camera Drivers**

Camera calibration is a very important step. Follow the steps on how to [calibrate](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) a Monocular Camera with raw images over ROS(this is called Camera Intrisic Calibration). Once done with the tutorial provided on the ROS wiki, follow the [Depth Camera Calibration](https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_pcl_ros/calibration.html)

When the camera calibration is finished we should have a depth error of at 1-2 cm between the the ros topic /camera_remote/depth_registered/points and /camera_remote_uncalibrated/depth_registered/points. The error can be visualized on rviz as such:

![image](https://user-images.githubusercontent.com/78880630/125126576-297f3300-e0b0-11eb-8721-7775b3713bad.png)

Open two terminals, in one terminal launch `roslaunch jsk_pcl_ros openni2_local.launch`, and in the other `roslaunch jsk_pcl_ros openni2_remote.launch`. This will run the Asus camera driver and load all the calibration files that were found in the camera calibration steps.

**Running Aubo Driver**

First make sure that the computer can connect to Aubo Robot. On the Aubo ipad, go to network settings->tap ifconfig:

![image](https://user-images.githubusercontent.com/78880630/125127557-8c24fe80-e0b1-11eb-884d-46a92c90292d.png)

In the image above insert the inet addr number as shown in the highlighted section into the space bellow Network debuging (this is the robots ip address). Then press ping and it will show 0% packet loss. 

Run the next command in the terminal to see if you can connect via the robot ip (in my case the ip is 192.168.1.101 this number will be different for everybody):

```
   ping 192.168.1.101
```

If it shows 0% packet loss in the terminal then the robot can be connected to via the robots ip.

To connect to the real robot through moveit in the same terminal run:

```
   roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=your-robot-ip 
```

Rviz will run with your robot model and the real robot will move just by dragging the endeffector arround and clicking Plan and execute in the Planning tab on Rviz.

![image](https://user-images.githubusercontent.com/78880630/125128574-0904a800-e0b3-11eb-8511-b4a146b5b47b.png)

**Cordinate Frames and Transformations**

ROS provides a very useful package to keep track of multple coordinate frames over time called [tf](http://wiki.ros.org/tf). To find the position of the robot base frame to the camera frame will be needed in order to have perform any grasping. To do this, Aruco Tags will be used. First you stick the aruco tag onto the end-effector of the robot. Have the tag visible to the camera and run in a terminal:
```
   roslaunch aruco_ros single.launch 
```
To view the results of the aruco marker launch file run in a new terminal:
```
   rosrun image_view image_view image:=/aruco_single/result
```
![image](https://user-images.githubusercontent.com/78880630/125129383-3bfb6b80-e0b4-11eb-9539-661f272b1f4e.png)

The aruco marker will create the transformation from the camera frame to the marker frame. This is where the static_transform_publisher will be useful. We can then find the transformation from the end effector frame to the base frame of the robot to get its position relative to the camera frame. To do this edit node with name="robot_base_broadcaster" in the file [static_frame.launch](https://github.com/nickhward/Aubo-Pick-And-Place/blob/main/static_frame.launch) and insert the the x y z rz ry rx where the numbers are in `args`. These values can be found directly from the teach pendant on the Aubo ipad. Then run the command in a new terminal: 

```
   roslaunch static_frame.launch
```

One can visualize the transformed frames in Rviz:

![image](https://user-images.githubusercontent.com/78880630/125131514-c0032280-e0b7-11eb-93fd-9d01d8149027.png)

To find the transformation between the camera frame and robot frame type in the a terminal:

```
   rosrun tf tf_echo camera_rgb_optical_frame base_link_calculated
```

Then create a node for static_Transform_publisher in the static_frame.launch file using the values provided by the tf_echo command (translation[x, y, z] and Quaternion[x, y, z, w]): 

```
   <node pkg="tf" type="static_transform_publisher" name="cam_broadcaster" args="   x  y  z  x  y  z  w  camera_rgb_optical_frame  base_link_calculated   100" /> 
```
**Object Detection State of the Art (YOLOv4)**

YOLOv4 will allow the computer to determine what objects are in the point cloud. Darknet Ros is a convenient way to get bounding boxes that are published on as a rostopic. Subscribing to the topic will allow easy access to continous bounding boxes even if the object is moved. To run YOLOv4 with ros it will be done by running in a terminal the command:

```roslaunch darknet_ros darknet_ros.launch```

A new window will pop up with bounding boxes around the object YOLO has been trained on. A full guide on how to train you own custom weights can be found at [this](https://github.com/AlexeyAB/darknet) GitHub repository.


**Cloud Clustering and Centroids**

The .cpp file [pass_through.cpp](https://github.com/nickhward/Aubo-Pick-And-Place/blob/main/pass_through.cpp) filters out the point cloud. It used a voxel filter, a statistical filter, and a filter to get rid of the table top. Then inorder to only obtain the objects I used a function called Euclidean Cluster Extraction. This will cluster each object, and we can compute the 3D centroid, then convert the centroid into 2D. The 2D Centroid will be compared to the center point of the bounding box from the rostopic(/darknet_ros/bounding_boxes). The object that was selected will have the shortest euclidean between the bounding box and the 2D centroid. 

![image](https://user-images.githubusercontent.com/78880630/125390026-03c78780-e357-11eb-9b8b-d30ff195a686.png)

To run the point cloud clustering and object selection; in a new terminal run:

```
   rosrun my_aubo_i5_robot_config pass_through
```

This will allow you to enter the object name you want to cluster. To visualize the point cloud in rviz, add the topic /output_cloud_filtered. You should see something similar to the images bellow.

![image](https://user-images.githubusercontent.com/78880630/125386289-df68ac80-e350-11eb-8eb8-923affaac243.png)
![image](https://user-images.githubusercontent.com/78880630/125386309-e7c0e780-e350-11eb-866c-573925a464a0.png)

**GPD_ROS**

We will be inserting the point cloud that is being published from pass_through.cpp.

Run in a new terminal:

```
   roslaunch gpd_ros ur5.launch
```

This will generate the best grasps on the selected object like in the image bellow.

![image](https://user-images.githubusercontent.com/78880630/125398043-a423a900-e363-11eb-9593-567a4f27458a.png)




**Picking Up Object**

With the Aubo Driver running from the command `roslaunch my_aubo_i5_robot_config moveit_planning_execution.launch robot_ip:=<your_robot_ip>` in a new terminal run:

```
   rosrun my_aubo_i5_robot_config pick_place
```

The real robot will then try to go to the grasp pose by finding ik solutions. If it says failed-No IK_solutions found just restart [pick_place](https://github.com/nickhward/Aubo-Pick-And-Place/blob/main/pick_place.cpp) until it works. If you have tried to restart it more than five times you may just have an unreachable grasp for your robot and will have to find another grasp that is reachable. 

## Installation Possible Problems and their Solutions
**Aubo_Driver**

Make sure to run: 
```
   sudo apt-get install ros-kinetic-ros-controllers 
   sudo apt-get install ros-kinetic-controller-manager 
```

One error when trying to launch the Aubo_Driver was: 
```
   error while loading shared libraries: libmoveit_robot_trajectory.so.0.9.15: cannot openshared object file: No such file or directory
```
The above error means that some of the moveit libraries are not seen by the program. To fix this run the following commands in the terminal(make sure to run each command in order):

```
   cd /opt/ros/kinetic/lib 

   sudo cp -r libmoveit_robot_trajectory.so.0.9.17 .. 

   sudo cp -r libmoveit_robot_model.so.0.9.17 .. 

   sudo cp -r libmoveit_robot_state.so.0.9.17 .. 

   cd .. 

   sudo mv libmoveit_robot_state.so.0.9.17 libmoveit_robot_state.so.0.9.15 

   sudo mv libmoveit_robot_model.so.0.9.17 libmoveit_robot_model.so.0.9.15 

   sudo mv libmoveit_robot_trajectory.so.0.9.17 libmoveit_robot_trajectory.so.0.9.15 

   sudo cp -r libmoveit_robot_state.so.0.9.15 lib/ 

   sudo cp -r libmoveit_robot_model.so.0.9.15 lib/ 

   sudo cp -r libmoveit_robot_trajectory.so.0.9.15 lib/
```


**GPD and gpd_ros**

GPD needs to be installed becuase it is needed for gpd_ros as a library. Gpd_ros is just a ros wrapper for GPD. 
When installing GPD in order to successfully make the project, go to the CMakeList.txt file and comment out the line:

```
   set(CMAKE_CXX_FLAGS "-O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
```

Uncomment the line:

```
   set(CMAKE_CXX_FLAGS "-fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17")
```

GPD_ROS can be ran using the command `roslaunch gpd_ros ur5.launch`, it will not work right away unfortunately.

In the ur5.launch file change the cloud_type to the point cloud type that will be used. Then change the cloud_topic to the correct rostopic that is needed to be used. The config_file value needs to be changed from:

```
   <param name="config_file" value="/home/ur5/projects/gpd/cfg/ros_vino_params.cfg" />
```
To:
```
<param name="config_file" value="/path/to/gpd/cfg/ros_eigen_params.cfg" />
```

The file ros_eigen_params.cfg is where most of the finctionality parameters are, such as setting the gripper dimensions, grasp workspaces, how many grasp samples are considered, etc. The only thing that needs to be changed in order for the launch file to work is the line:

```
   weights_file = /home/andreas/projects/gpd/lenet/15channels/params/
```

To: 

```
   weights_file = /path/to/gpd/lenet/15channels/params/
```
If GPD is running slow on producing grasps for you robot, I suggest lowering the parameter `num_samples = 500` to a number lower than 100 and change the parameter `workspace_graps = -1 1 -1 1 -1 1` to fit exactly the areas you need to grasp. 













