#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Import for publishing boolean messages
from filterpy.kalman import KalmanFilter
import numpy as np
import math

class LidarKalmanFilter:
    def __init__(self):
        rospy.init_node('lidar_kalman_filter', anonymous=True)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.state_pub = rospy.Publisher('/lidar_state', Bool, queue_size=10)  # Initialize publisher
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.initialize_kalman_filter()

    def initialize_kalman_filter(self):
        self.kf.F = np.array([[1., 1.], [0., 1.]])  # State transition matrix
        self.kf.H = np.array([[1., 0.]])  # Measurement function
        self.kf.R = np.array([[0.5]])  # Measurement noise
        self.kf.Q = np.eye(2) * 0.001  # Process noise
        self.kf.P *= 1000.  # Error covariance matrix
        self.kf.x = np.array([[0.], [0.]])  # Initial state

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        points = []

        for i, range_val in enumerate(ranges):
            if not np.isinf(range_val):  # Ignore infinite values
                angle = angle_min + i * angle_increment
                self.kf.predict()
                self.kf.update(np.array([[range_val]]))
                filtered_range = self.kf.x[0][0]
                x = filtered_range * np.cos(angle)
                y = filtered_range * np.sin(angle)
                angle_degrees = angle * 180 / math.pi
                points.append([x, y, angle_degrees])

        # Filter points within specified range and y > 0
        filtered_points = [point for point in points if point[2] < 95 and point[2] > 63.43]
        for point in filtered_points:
            print(f"Filtered Point: x={point[0]}, y={point[1]}, angles = {point[2]}")

        # Check if all filtered_points have y > 0.1
        if all(point[1] > 0.1 for point in filtered_points) and filtered_points:
            # Publish True to the /lidar_state topic
            self.state_pub.publish(True)
            point_with_min_x = min(filtered_points, key=lambda p: p[0])
            print(f"Point with smallest x within range: {point_with_min_x}")
        else:
            # Optionally publish False if the criteria are not met
            self.state_pub.publish(False)
            print("No points meet the criteria.")

if __name__ == '__main__':
    try:
        lkf = LidarKalmanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
