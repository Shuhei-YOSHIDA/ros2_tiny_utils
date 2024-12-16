rqt_progressbar
====

## rqt\_progressbar
### What's this
* ROS2 RQT plugin
* Shows progress of replaying of bag file by using `/clock` topic.
* Call `/rosbag2_player/seek` service by clicking on progress bar

### How to use
* After `colcon build`, you need to execute `rqt --force-discover` to use this RQT plugin for the first time.
* Set the start and the end of Unix-time of your rosbag file.
* Play rosbag file such as `$ ros2 bag play --clock <Hz> <your rosbag>`
* Click on progress-bar in order to seek the bag file.

### Note
* The rosbag playing node will crash if the target time for seeking is outside of the recording.

### Sample
YouTube

[![Youtube](http://img.youtube.com/vi/4WgkY-Hzbl0/0.jpg)](https://www.youtube.com/watch?v=4WgkY-Hzbl0)
