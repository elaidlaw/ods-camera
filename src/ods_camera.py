#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import LinkStates
import tf
from cv_bridge import CvBridge
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image as PImage
import cv2

WIDTH = 2000
HEIGHT = 1000

camera1_yaws = []
camera2_yaws = []
left_image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
right_image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
left_publish = None
right_publish = None
last_yaw_1 = 0
last_yaw_2 = 0
index = -1

def link_states_subscriber(data):
    global index
    if index == -1:
        index = data.name.index('ods_camera::spinner')
    if index != -1:
        orientation = data.pose[index].orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        camera1_yaws.append((rospy.get_rostime().to_sec(), euler[2]))
        camera2_yaws.append((rospy.get_rostime().to_sec(), euler[2]))

def camera1_subscriber(data):
    global left_image, last_yaw_1, left_publish
    image_time = data.header.stamp.to_sec()
    # rospy.loginfo(str(image_time) + ' 1')
    current = camera1_yaws[0]
    while len(camera1_yaws) > 1 and camera1_yaws[0][0] < image_time:
        current = camera1_yaws[0]
        camera1_yaws.pop(0)
    if camera1_yaws[0][0] == current[0]:
        actual_yaw = current[1]
    else:
        actual_yaw = (image_time - current[0]) / (camera1_yaws[0][0] - current[0]) * (camera1_yaws[0][1] - current[1]) + current[1]
    # rospy.loginfo(str(actual_yaw))
    image = CvBridge().imgmsg_to_cv2(data).transpose([1, 0, 2])
    center_column = int((actual_yaw + math.pi) / (2 * math.pi) * WIDTH)
    left = max(center_column, 5) - 5
    right = min(center_column, WIDTH - 10) + 10
    left_offset = center_column - left
    right_offset = right - center_column
    slice = cv2.resize(image[:, -int(left_offset / 1.5) + 500:int(right_offset / 1.5) + 500, :], (right - left, HEIGHT))
    left_image[:, left:right, :] = slice
    if last_yaw_1 > 0 and actual_yaw < 0:
        left_publish = left_image.copy()
    last_yaw_1 = actual_yaw



def camera2_subscriber(data):
    global right_image, last_yaw_2, right_publish
    image_time = data.header.stamp.to_sec()
    # rospy.loginfo(str(image_time) + ' 1')
    current = camera2_yaws[0]
    while len(camera2_yaws) > 1 and camera2_yaws[0][0] < image_time:
        current = camera2_yaws[0]
        camera2_yaws.pop(0)
    if camera2_yaws[0][0] == current[0]:
        actual_yaw = current[1]
    else:
        actual_yaw = (image_time - current[0]) / (camera2_yaws[0][0] - current[0]) * (camera2_yaws[0][1] - current[1]) + current[1]
    # rospy.loginfo(str(actual_yaw))
    image = CvBridge().imgmsg_to_cv2(data).transpose([1, 0, 2])
    center_column = int((actual_yaw + math.pi) / (2 * math.pi) * WIDTH)
    left = max(center_column, 5) - 5
    right = min(center_column, WIDTH - 10) + 10
    left_offset = center_column - left
    right_offset = right - center_column
    slice = cv2.resize(image[:, -int(left_offset / 1.5) + 500:int(right_offset / 1.5) + 500, :], (right - left, HEIGHT))
    right_image[:, left:right, :] = slice
    if last_yaw_2 > 0 and actual_yaw < 0:
        right_publish = right_image.copy()
    last_yaw_2 = actual_yaw

def main():
    global left_publish, right_publish
    rospy.init_node('ods_camera', anonymous=False)
    rospy.Subscriber('gazebo/link_states', LinkStates, link_states_subscriber)
    rospy.Subscriber('ods_camera/camera1/image_raw', Image, camera1_subscriber)
    rospy.Subscriber('ods_camera/camera2/image_raw', Image, camera2_subscriber)

    pub = rospy.Publisher('ods_camera/ods_image', Image, queue_size=1)
    rate = rospy.Rate(.3)
    while not rospy.is_shutdown():
        if left_publish != None and right_publish != None:
            left_publish = None
            right_publish = None
            pub.publish(CvBridge().cv2_to_imgmsg(np.append(left_image, right_image, axis=0), encoding='rgb8'))
        rate.sleep()

if __name__ == '__main__':
    main()
