# nuturtle_control ROS package

### Brief overview 
The package involves three nodes:
* turtle_interface: provides an interface between command source and wheel control
* odometry: calculate the transform between odometry frame and body frame, and broadcast it
* circle: one of the command source to cause the turtlebot to move in a circle

### Experiments

1. Drive the robot forward and backward in a straight line 
  * Initial odometry:
    position: 
      x: 0.1141<br>
      y: 0.01047<br>
      z: 0.0<br>
    orientation: 
      x: -0.0<br>
      y: 0.0<br>
      z: 0.0187<br>
      w: -0.999<br>
  * Final odometry:
    position: 
      x: 0.0967<br>
      y: 0.01445<br>
      z: 0.0<br>
    orientation: 
      x: -0.0<br>
      y: 0.0<br>
      z: 0.01720<br>
      w: -0.999<br>
  * Rviz: https://youtu.be/CyYhnOoOMoc
  * Real: https://youtu.be/4RChpvIqmVU

2. Rotate the robot clockwise and counter clockwise 
  * Initial odometry:
    position: 
      x: 0.0967<br>
      y: 0.01445<br>
      z: 0.0<br>
    orientation: 
      x: -0.0<br>
      y: 0.0<br>
      z: 0.01720<br>
      w: -0.999<br>
  * Final odometry:
    position: 
      x: 0.0962<br>
      y: 0.01479<br>
      z: 0.0<br>
    orientation: 
      x: 0.0<br>
      y: 0.0<br>
      z: -0.1611<br>
      w: 0.986<br>
  * Rviz: https://youtu.be/3sMmMYYVfFk
  * Real: https://youtu.be/ljfErUOTZg4

3. Drive the robot in a circle, clockwise and counter clockwise
  * Initial odometry: 
    position: 
      x: 0.0<br>
      y: 0.0<br>
      z: 0.0<br>
    orientation: 
      x: 0.0<br>
      y: 0.0<br>
      z: 0.0<br>
      w: 1.0<br>
  * Final odometry:
    position: 
      x: 0.0985<br>
      y: 0.0903<br>
      z: 0.0<br>
    orientation: 
      x: 0.0<br>
      y: 0.0<br>
      z: 0.661<br>
      w: 0.749<br>
  * Rviz: https://youtu.be/bjnNbjgbFXo
  * Real: https://youtu.be/Ph9QovCfH0M

4. Try drive the robot in a circle, and get a significantly better result
  * Initial odometry:
    position: 
      x: 0.0<br>
      y: -2.274e-18<br>
      z: 0.0<br>
    orientation: 
      x: 0.0<br>
      y: 0.0<br>
      z: 0.0<br>
      w: 1.0<br>
  * Final odometry:
    position: 
      x: 0.0608<br>
      y: 0.0633<br>
      z: 0.0<br>
    orientation: 
      x: 0.0<br>
      y: 0.0<br>
      z: 0.558<br>
      w: 0.829<br>
  * Rviz: https://youtu.be/VSdJZ1Za5ss
  * Real: https://youtu.be/xIeY5_YXUVws
  * The driving did result in a better final pose. I drove the turtlebot with slower speed and shorter distance. This is because the more distance or higher speed I drive the turtlebot, the more odometry error will be accumulated because of wheel slips. 

### User instructions
```shell
roslaunch nuturtle_control start_robot.launch
```
* Arguments:
  * cmd_src: command sources, default to be circle
    * circle: drive in a circle with given angular velocity and radius
    * teleop: use turtlebot3_teleop_key node
    * none: start nothing
  * robot: robot to be controled, default to be nusim
    * nusim: start nusim simulator
    * localhost: run the nodes directly from turtlebot3
    * <turtlebotname>: the name of any turtlebot, to run the nodes on that turtlebot
    * none: do not launch any addition nodes
  * use_rviz: whether to launch rviz



