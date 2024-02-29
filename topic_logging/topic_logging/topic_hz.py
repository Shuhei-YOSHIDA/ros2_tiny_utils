"""
File: topic_hz.py
Author: Shuhei-YOSHIDA
Description: Based on ros2cli/ros2topic/ros2topic/verb/hz.py.
             Calculate frequency of published topic
"""

from collections import defaultdict

import functools
import math
import threading

import argparse

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data

from ros2topic.api import get_msg_class


class TopicHz(Node):
    def __init__(self, topic, window_size, use_wtime=False):
        super().__init__('topic_hz')
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
        self._last_printed_tn = defaultdict(int)
        self._msg_t0 = defaultdict(lambda: -1)
        self._msg_tn = defaultdict(int)
        self._times = defaultdict(list)
        self.use_wtime = use_wtime  # Use Walltime when clock is not published

        self.window_size = window_size

        self._clock = self.get_clock()

        msg_class = get_msg_class(self, topic, blocking=True, include_hidden_topics=True)
        if msg_class is None:
            self.destroy_node()
            return
        self.create_subscription(
                msg_class,
                topic,
                functools.partial(self.callback_topic, topic=topic),
                qos_profile_sensor_data)

    def get_last_printed_tn(self, topic=None):
        if topic is None:
            return self.last_printed_tn
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        if topic is None:
            self.last_printed_tn = value
        self._last_printed_tn[topic] = value

    def get_msg_t0(self, topic=None):
        if topic is None:
            return self.msg_t0
        return self._msg_t0[topic]

    def set_msg_t0(self, value, topic=None):
        if topic is None:
            self.msg_t0 = value
        self._msg_t0[topic] = value

    def get_msg_tn(self, topic=None):
        if topic is None:
            return self.msg_tn
        return self._msg_tn[topic]

    def set_msg_tn(self, value, topic=None):
        if topic is None:
            self.msg_tn = value
        self._msg_tn[topic] = value

    def get_times(self, topic=None):
        if topic is None:
            return self.times
        return self._times[topic]

    def set_times(self, value, topic=None):
        if topic is None:
            self.times = value
        self._times[topic] = value

    def callback_topic(self, m, topic=None):
        with self.lock:
            curr_rostime = self._clock.now() if not self.use_wtime \
                    else Clock(clock_type=ClockType.SYSTEM_TIME).now()

            # time reset
            if curr_rostime.nanoseconds == 0:
                if len(self.get_times(topic=topic)) > 0:
                    print('time has reset, resetting counters', flush=True)
                    self.set_times([], topic=topic)
                return

            curr = curr_rostime.nanoseconds
            msg_t0 = self.get_msg_t0(topic=topic)
            if msg_t0 < 0 or msg_t0 > curr:
                self.set_msg_t0(curr, topic=topic)
                self.set_msg_tn(curr, topic=topic)
                self.set_times([], topic=topic)
            else:
                self.get_times(topic=topic).append(curr - self.get_msg_tn(topic=topic))
                self.set_msg_tn(curr, topic=topic)

            if len(self.get_times(topic=topic)) > self.window_size:
                self.get_times(topic=topic).pop(0)

    def get_hz(self, topic=None):
        if not self.get_times(topic=topic):
            return
        elif self.get_last_printed_tn(topic=topic) == 0:
            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)
            return
        elif self.get_msg_tn(topic=topic) < self.get_last_printed_tn(topic=topic) + 1e9:
            return

        with self.lock:
            times = self.get_times(topic=topic)
            n = len(times)
            mean = sum(times) / n
            rate = 1. / mean if mean > 0. else 0

            std_dev = math.sqrt(sum((x - mean)**2 for x in times) / n)

            max_delta = max(times)
            min_delta = min(times)

            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)

        return self.get_last_printed_tn(topic=topic), rate, min_delta, max_delta, std_dev, n

    def print_hz(self, topic=None):
        ret = self.get_hz(topic)
        if ret is None:
            return
        time, rate, min_delta, max_delta, std_dev, window = ret
        print('%.3f, %.3f, %.3f, %.3f, %.5f, %s'
              % (time/1e9, rate*1e9, min_delta*1e-9, max_delta*1e-9, std_dev*1e-9, window),
              flush=True)
        return


def main(args=None):
    rclpy.init()
    argv = rclpy.utilities.remove_ros_args()
    argv.pop(0)

    parser = argparse.ArgumentParser()
    parser.add_argument('topic_name', help="topic name")
    parser.add_argument('-w', '--window', dest="window_size", type=int,
                        help="window size, default %d" % 30, default=30)
    args = parser.parse_args(argv)

    topic = args.topic_name
    window = args.window_size
    topic_hz = TopicHz(topic, window)

    # rclpy.spin(topic_hz)
    print("time[s], rate[hz], min_delta[s], max_delta[s], std_dev[s], window", flush=True)
    while rclpy.ok():
        rclpy.spin_once(topic_hz)
        topic_hz.print_hz(topic)

    topic_hz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(args)
