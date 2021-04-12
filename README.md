# limit_cycle_walking
Hybrid Walking Pattern Generator for bipedal robots, merging the Virtual Constraints and the Preview Control theories

To run the WPG, the pre-required software can be found here: http://54.73.207.169/nightly/

    1. download and install the debian: xbot1-full-devel <br />
    2. clone and install the repo: https://github.com/ADVRHumanoids/OpenMpC <br />

Steps to run the node:

    I. run roscore <br />
    II. run gazebo <br />
    III. homing robot <br />
    IV. run rosrun limiit_cycle_walking walking_server <br />
    V. call rosservice call /walker/run 1 to walk!!

