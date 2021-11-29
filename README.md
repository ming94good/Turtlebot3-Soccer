# turtlebot3-soccer
ROS, OpenCV, Python

## Overview
Turtlebot kicks a green ball.
<img src="images/soccer.gif" width="200" height="300" />
## Prerequisite
1. ROS (noetic in my case)
2. [Turtlebot3 packages complete installation](https://github.com/ROBOTIS-GIT/turtlebot3)
3. OpenCV (4.2.0 in my case)
4. Python3 (3.8.10 in my case)
4. Python packages
```
pip install -r requirements.txt
```

## Run Code
1. Ball Tracking
```
python3 ball_tracking.py
```
2. Linear Move Control
```
python3 linear_move.py
```
3. Rotate Move Control
```
python3 rotate_angle.py
```
4. View Output
```
rqt_image_view
```
![output](images/output.gif)

## Reference
https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision/wiki/Robot-control-with-colored-object-detection
 