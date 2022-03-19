# nuslam ROS package

### Brief overview 
A package to perform extended Kalman Filter SLAM algoritm on turtlebot both in simulation and real world

### Launch files

* To implement EKF SLAM in simulation (Rviz) with known data association, run the following command in the terminal:
```shell
roslaunch nuslam nuslam.launch
```
* To implement EKF SLAM in simulation (Rviz) with unknown data association, run the following command in the terminal:
```shell
roslaunch nuslam unknown_data_assoc.launch
```
* Run the following command for the real robot:
```shell
roslaunch nuslam unknown_data_assoc.launch robot:=<robot name>
```
* To test the landmark detection algoritm, run the following command in the terminal:
```shell
roslaunch nuslam landmark_detect.launch
```
* Run the following command for the real robot:
```shell
roslaunch nuslam landmark_detect.launch robot:=<robot name>
```



### Results

#### Video demo

The following video demostrate the implementation of EKF SLAM with unknown data association in Simulation:<br>
https://youtu.be/tkifSmn2UG8

#### Pose error

* The pose of the actural robot in [X,Y,Z,R,P,Y]:<br>
[0.050, -0.090, 0.010, 0.000, 0.000, -170.976]
* The pose of the odometry robot in [X,Y,Z,R,P,Y]:<br>
[0.006, -0.076, 0.010,0.000, 0.000, -174.998]
* The pose of the SLAM robot in [X,Y,Z,R,P,Y]:<br>
[0.049, -0.090, 0.010, 0.000, 0.000, -170.990]<br>

* The pose error between real robot and odometry robot in [X,Y,Z,R,P,Y]: <br>
[-0.044, 0.014, 0.00, 0.00, 0.00, -4.022]
* The pose error between real robot and SLAM robot in [X,Y,Z,R,P,Y]: <br>
[-0.001, 0.000, 0.00, 0.00, 0.00, -0.014]







