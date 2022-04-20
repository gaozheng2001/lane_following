# lane following
This is the lab 4 project for SUSTech EE346.

# Usage

## 1. Clone the source code
```
cd ~/catkin_ws/src
  
git clone https://github.com/gaozheng2001/lane_following.git
```

## 2. Catkin make the lane following package
```
cd ..
  
catkin_make
```

## 3. Add course models
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/lane_following/models
```
   
## 4. Launch the gazebo map
```
source ~/catkin_ws/devel/setup.bash
   
roslaunch lane_following race_track.launch 
```

## 5. Run lane following python node
```
cd ~/catkin_ws/src/lane_following/scripts/
   
chmod +x lane_following.py
  
cd ~/catkin_ws

source devel/setup.bash

rosrun lane_following lane_following.py
```
![image](https://github.com/gaozheng2001/lane_following/blob/main/data/demo.gif)

## 6. Run part1 python node
This part avoids the phenomenon of identifying the wrong lane line by changing the threshold when the image is converted to the HSV color gamut.
```
cd ~/catkin_ws/src/lane_following/scripts/

chmod +x lane_following_part1.py

cd ~/catkin_ws

source devel/setup.bash

rosrun lane_following lane_following.py
```

## 7. Run part2 python node
Add the homography transformation to the camera basis on part1.
```
cd ~/catkin_ws/src/lane_following/scripts/

chmod +x lane_following_part2.py

cd ~/catkin_ws

source devel/setup.bash

rosrun lane_following lane_following.py
```

## 8. Run part3 python node
Add the recognition of ARUCO code to achieve the function of stopping at a specified distance.
```
cd ~/catkin_ws/src/lane_following/scripts/

chmod +x lane_following_part3.py

cd ~/catkin_ws

source devel/setup.bash

rosrun lane_following lane_following.py
```