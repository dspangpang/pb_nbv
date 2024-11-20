#!/usr/bin/env python

import rospy
import tf2_ros
import tf
import numpy as np

def get_transform_matrix(source_frame, target_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # 等待变换可用
        tf_buffer.can_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1.0))
        transform = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
        
        # 从变换中提取平移和旋转
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # 将四元数转换为旋转矩阵
        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        r = tf.transformations.quaternion_matrix(q)

        # 创建4x4的变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = r[:3, :3]
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        return transform_matrix

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Transform not available")
        return None

if __name__ == "__main__":
    rospy.init_node('transform_listener')

    source_frame = "base"
    target_frame = "d435_depth_optical_frame"

    transform_matrix = get_transform_matrix(source_frame, target_frame)
    if transform_matrix is not None:
        print("Transform matrix from {} to {}:".format(source_frame, target_frame))
        print(transform_matrix)
        # 保存到文件
        np.savetxt("transform_matrix.txt", transform_matrix)
    else:
        print("Failed to get transform matrix")