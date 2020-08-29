# -*- coding: utf-8 -*-
"""
Created on 2019/4/17

@author: ni
"""

from __future__ import print_function
import roslib
roslib.load_manifest("pick_brick_ur5")
import sys, math, cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rospy_tutorials.msg import Floats
from cv_bridge import CvBridge, CvBridgeError


C_PARAM = [320/math.tan(math.pi/6), 320/math.tan(math.pi/6), 320, 240]  # 相机4个内参

class brick_detection:
    def __init__(self):
        self.image_pub = rospy.Publisher("brick_position", Floats,queue_size=1)
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, self.callback)
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_raw", Image, \
            self.callback_depth)
        self.depth_flag = False
 
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
 
        img_position = self.brick_detect(cv_image)
        while not self.depth_flag:
            pass
        self.depth_flag = False
        brick_depth = self.depth_img[img_position[1], img_position[0]]
        c_para = C_PARAM  # 相机4个内参
        brick_world = self.depth2world(img_position, brick_depth, c_para)

        # print(img_position, brick_depth, c_para)
        cv2.circle(cv_image, tuple(img_position), 5, (255, 0, 0))  # 画圆
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            position_ = brick_world
            rospy.loginfo(position_)
            self.image_pub.publish(position_)
        except CvBridgeError as e:
            print(e)

    def callback_depth(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        if False:  # 保存图片时使用
            depth_image[np.isnan(depth_image)] = 0
            depth_image = 255*depth_image/np.max(depth_image)
            depth_image = depth_image.astype(np.uint8)
            cv2.imwrite("1_depth.png", depth_image)  # 保存测试图片
            cv2.imshow("depth window", depth_image)
            cv2.waitKey(3)

        self.depth_img = depth_image
        self.depth_flag = True

    def brick_detect(self, img):  # 砖块检测算法
        img = img.copy()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转hsv颜色空间

        z_hsv = np.array([9, 194, 57])  # 砖块hsv值需要标定
        lower = np.array([2, 170, 37])
        upper = np.array([29, 220, 110])
        z_mask = cv2.inRange(hsv, lower, upper)  # 通过颜色提取砖块的掩膜

        if True:  # 掩膜形态学处理
            hsv_mask = z_mask
            elem=cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
            z_open = cv2.morphologyEx(hsv_mask, cv2.MORPH_OPEN, elem)
            elem=cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(7, 7))
            z_out = cv2.morphologyEx(z_open, cv2.MORPH_CLOSE, elem)
        else:
            hsv_mask = z_mask
            z_out = hsv_mask

        _, contours, hierarchy = cv2.findContours(z_out, cv2.RETR_EXTERNAL, \
            cv2.CHAIN_APPROX_SIMPLE)  # 从掩膜图像提取轮廓, cv2.CHAIN_APPROX_NONE

        c_max = []
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            # 处理掉小的轮廓区域，这个区域的大小自己定义
            if(area < (30*30)):
                continue
            else:
                c_max.append(cnt)

        z_center = [0, 0]
        for i in range(len(c_max)):
            im_temp = cv2.drawContours(img.copy(), [c_max[i]], -1, (0, 0, 255), \
                thickness=1)  # 画出轮廓图

            z_rect = cv2.minAreaRect(c_max[i])  # 获取最小外接矩形顶点坐标
            z_rect = cv2.boxPoints(z_rect)
            z_rect = z_rect.astype(np.int32)

            epsilon = 0.01 * cv2.arcLength(c_max[i],True)
            z_cor = cv2.approxPolyDP(c_max[i], epsilon, True)  # 获取拟合多边形顶点坐标

            z_box = np.zeros([4, 2], dtype=np.int32)
            d_cor = np.array(z_cor)
            d_cor = d_cor.reshape((-1, 2))
            if len(d_cor)>4:  # 拟合多边形顶点过多，选择有效顶点
                for j in range(len(z_rect)):
                    d_rect = np.array([z_rect[j] for _ in range(len(d_cor))])
                    n_cor = np.multiply(d_cor-d_rect, d_cor-d_rect)
                    n_cor = np.argmin(n_cor[:, 0]+n_cor[:, 1])
                    z_box[j] = d_cor[n_cor]
                    d_cor = np.delete(d_cor, n_cor, 0)
            elif len(d_cor)==4:  # 拟合多边形顶点满足四边形
                z_box = d_cor
            elif len(d_cor)<4:  # 拟合多边形顶点过少，放弃该轮廓
                continue

            for j in range(len(z_box)):
                z_center += z_box[j]
            z_center = z_center//4  # 计算砖块中心像素坐标

        position_ = z_center
        return position_

    def depth2world(self, point, depth, c_para):
        """parameter:
        point: (x, y) 像素坐标
        depth: d 深度值
        c_para： (fx, fy, cx, cy) 相机内参
        """
        world_p = [0, 0, 0, 1]
        world_p[0] = depth*(point[0]-c_para[2])/c_para[0]
        world_p[1] = depth*(point[1]-c_para[3])/c_para[1]
        world_p[2] = depth
        return world_p
 

def main(args):
    ic = brick_detection()
    rospy.init_node("brick_detection", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 

if __name__ == '__main__':
    main(sys.argv)
