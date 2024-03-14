topic_logging
====

## topic_hz
Almost same as `$ ros2 topic hz`, however this outputs CSV-like string.

Example:
```
$ ros2 run topic_logging topic_hz /chatter
time[s], rate[hz], min_delta[s], max_delta[s], std_dev[s], window
1710399028.973, 9.999, 0.100, 0.100, 0.00005, 11
1710399030.073, 10.000, 0.100, 0.100, 0.00007, 22
1710399031.073, 10.000, 0.100, 0.100, 0.00007, 30
1710399032.073, 10.000, 0.100, 0.100, 0.00007, 30
1710399033.073, 10.000, 0.100, 0.100, 0.00006, 30
```

You can connect output to pipe or redirect.
```
$ ros2 run topic_logging topic_hz /sampled_topic | tee sample.csv
$ ros2 run topic_logging topic_hz /sampled_topic > sample.csv
```

This code is based on [ros2/ros2cli (hz.py)](https://github.com/ros2/ros2cli/blob/1a4766098f917819508ce8b06ca44bdd041f5fc3/ros2topic/ros2topic/verb/hz.py), which is distributed under Apache-2.0 License.
