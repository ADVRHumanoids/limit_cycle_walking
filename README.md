# limit_cycle_walking
Hybrid Walking Pattern Generator for bipedal robots, merging the Virtual Constraints and the Preview Control theories

To run the WPG, the pre-required software can be found here: http://54.73.207.169/nightly/ 
    1. download and install the debian: xbot1-full-devel 
    2. clone and install the repo: https://github.com/ADVRHumanoids/OpenMpC
    
I. run roscore
II. run gazebo
III. homing robot
IV. run rosrun limiit_cycle_walking walking_server 
V. call rosservice call /walker/run 1 to walk!!

