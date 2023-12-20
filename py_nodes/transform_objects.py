#!/usr/bin/env python3

import numpy as np
# from np import array
import math

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# Cameras intrinsic parameters [K] (f~100px, w~640px, h~360px)
#  <fx>100</fx><fy>100</fy><cx>640</cx><cy>360</cy>
focal = 100.0
width_pixels = 640.0
height_pixels =320.0

# Pose of object in camera frame
object_x = width_pixels/2
object_y = height_pixels/2
# object_depth = pixel
object_depth = 0.719721


# Pose of Camera in world frame (from tf2_ros)
# TODO: get from topic/rviz
camera_pose_x = 0.5
camera_pose_y = 0.0
camera_pose_z = 0.5

# From Gazebo; pose of object
# x 1.665320
# y -0.232940
# z 0.510000
# Depth=pixel = 0.719721

"""
Funtion to do homogenous transformation of object pose (center pixels)
into world frame
"""
def transform_object_pose(object_x, object_y, object_depth):
    """
    """
    # Cameras intrinsic parameters [K] (f~100px, w~640px, h~360px)
    #  <fx>100</fx><fy>100</fy><cx>640</cx><cy>360</cy>
    K = np.array(
        [[focal, 0.,width_pixels/2],
        [0.,focal,  height_pixels/2],
        [0.,0.,1.]]
    )

    # No rotation occurs between camera and object
    R = np.identity(3)
    # R = np.zeros((3,3))
    # print(R)
    # print(R.shape)


    # translation matrix
    # depth is in x direction, pixel x is in y direction,
    # pixel y is in -x direction
    t = np.array([[object_x],[object_y],[object_depth]])
    # print(t)
    # print(t.shape)

    # Transformation Matrix
    # bottom = np.array([[0,0,0,1]])
    # trans = np.concatenate((R, t), axis=1)
    T = np.concatenate((R, t), axis=1)
    # T = np.concatenate((trans, bottom), axis=0)
    # print(trans)
    # print(T)
    # print(T.shape)

    # Origin in World Coordinate system, located at (X,Y,Z,1) = (0, 0, 0, 1)
    Pw = np.array([[camera_pose_x],[camera_pose_y],[camera_pose_z],[1]])
    print("Point in world coordinates: ",Pw)

    # Project (Xw, Yw, Zw, 0) into cameras coordinate system
    Pc = np.matmul(T, Pw)
    print("Point in camera coordinates: ", Pc)

    # Apply camera intrinsics to map (Xc, Yc, Zc) to p=(x, y, z)
    p = np.matmul(K, Pc)
    print("Map point in camera world to p: ", p)
    # print(p.shape)

    # Normalize by z to get (u,v,1)
    uv = (p / p[2][0])[:-1]
    print(uv)
    # print(uv.shape)

    return uv[0], uv[1]

world_x, world_y = transform_object_pose(object_x, object_y, object_depth)

print("Object pose in world frame is (x, y): ( ", world_x, ", ", world_y, ")\n")


# class ComputeTransformation(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             'front_realsense_depth/depth/image_raw',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('I heard: "%s"' % msg.data)

# def main(args=None):
#     rclpy.init(args=args)

#     transformation_computer = ComputeTransformation()

#     rclpy.spin(transformation_computer)


#     transformation_computer.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()