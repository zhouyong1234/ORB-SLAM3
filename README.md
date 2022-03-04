# ORB-SLAM3
ORB-SLAM3添加ROS接口

# How to Run

```
mkdir -p orb_ws/src
cd orb_ws/src
git clone git@github.com:zhouyong1234/ORB-SLAM3.git
cd ..
catkin_make
source devel/setup.bash
rosrun ORB_SLAM3 mono_avi
```