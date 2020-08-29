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

from geometry_msgs.msg import PoseStamped
import tf


C_PARAM = [320/math.tan(math.pi/6), 320/math.tan(math.pi/6), 320, 240]  # 相机4个内参
ERR_POS = [[0 , 0, 0, 0],[0 , 0, 0, 0], [0 , 0, 0, 0], [0 , 0, 0, 0]]

class brick_detection:
    def __init__(self):
        self.image_pub = rospy.Publisher("brick_position", Floats,queue_size=1)
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image, self.callback)
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_raw", Image, \
            self.callback_depth)
        self.depth_flag = False
        self.tflistener = tf.TransformListener()
 
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img_position = self.brick_detect(cv_image)

        while not self.depth_flag:
            continue
        self.depth_flag = False
        brick_world = 0
        if type(img_position)==int:
            brick_world = ERR_POS
            brick_base = ERR_POS
        else:
            brick_depth = []
            for i in img_position:
                brick_depth.append(self.depth_img[i[1], i[0]])
            c_para = C_PARAM  # 相机4个内参
            brick_world = self.depth2world(img_position, brick_depth, c_para)
            brick_base = self.brick2base(brick_world)
            cv2.circle(cv_image, tuple(img_position[0]), 5, (255, 0, 0))  # 画圆

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        deep_img = self.depth_img.copy()
        cv2.imshow("deep_img window", deep_img)
        cv2.waitKey(3)

        try:
            position_ = brick_base
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
        lower = np.array([2, 150, 37])
        upper = np.array([29, 240, 110])
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
            if(area < (50*50)):
                continue
            else:
                c_max.append(cnt)
        if len(c_max)==0:
            return 0

        # self.brick_select()  # 选择砖块的决策

        cnt_used = c_max[0]
        cnt_used = cv2.convexHull(cnt_used)  # 找到凸形外接框

        z_rect = cv2.minAreaRect(cnt_used)  # 获取最小外接矩形顶点坐标
        z_rect = cv2.boxPoints(z_rect)
        z_rect = z_rect.astype(np.int32)

        epsilon = 0.05 * cv2.arcLength(cnt_used,True)
        z_cor = cv2.approxPolyDP(cnt_used, epsilon, True)  # 获取拟合多边形顶点坐标

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
            return 0

        # 调整砖块4个角点为标准顺序
        z_box_order = np.zeros_like(z_box)
        z_temp = [640, 480]
        ind_ = -1
        for i, (p_,) in enumerate(zip(z_box)):
            if z_temp[1]>p_[1]:
                ind_ = i
                z_temp = p_
            elif z_temp[1]==p_[1] and z_temp[0]>p_[0]:
                ind_ = i
                z_temp = p_
        z_box_order[0] = z_box[ind_]
        z_temp = z_box
        z_temp = np.delete(z_temp, ind_, axis=0)
        z_temp1 = np.array([z_box[ind_] for _ in range(len(z_temp))])
        z_temp1 = z_temp-z_temp1
        for i in range(len(z_temp)):
            if z_temp1[i, 0]==0:
                z_temp1[i, 0]=1
        z_temp1 = [[i, math.atan2(z_temp1[i, 1], z_temp1[i, 0])] \
            for i in range(len(z_temp1))]
        def sort_k(data_):
            return data_[1]
        z_temp1.sort(key=sort_k)
        for i in range(len(z_temp)):
            z_box_order[i+1] = z_temp[z_temp1[i][0]]
        z_temp1 = z_box_order[0:2]-z_box_order[1:3]
        if (z_temp1[0, 0]**2+z_temp1[0, 1]**2)<(z_temp1[1, 0]**2+z_temp1[1, 1]**2):
            z_temp1 = z_box_order[0].copy()
            for i in range(len(z_box_order)-1):
                z_box_order[i] = z_box_order[i+1]
            z_box_order[3] = z_temp1

        # 计算砖块中心像素坐标
        z_center = [0, 0]
        if (z_box_order[0, 0]-z_box_order[2, 0])==0 and (z_box_order[1, 0]-z_box_order[3, 0])==0:
            return 0
        elif (z_box_order[0, 0]-z_box_order[2, 0])==0:
            z_center[0] = z_box_order[0, 0]
            k_ = (z_box_order[1, 1]-z_box_order[3, 1])/(z_box_order[1, 0]-z_box_order[3, 0])
            z_center[1] = k_*(z_center[0]-z_box_order[1, 0])+z_box_order[1, 1]
        elif (z_box_order[1, 0]-z_box_order[3, 0])==0:
            z_center[0] = z_box_order[1, 0]
            k_ = (z_box_order[0, 1]-z_box_order[2, 1])/(z_box_order[0, 0]-z_box_order[2, 0])
            z_center[1] = k_*(z_center[0]-z_box_order[0, 0])+z_box_order[0, 1]
        else:
            k1_ = 1.0*(z_box_order[0, 1]-z_box_order[2, 1])/(z_box_order[0, 0]-z_box_order[2, 0])
            k2_ = 1.0*(z_box_order[1, 1]-z_box_order[3, 1])/(z_box_order[1, 0]-z_box_order[3, 0])
            z_center[0] = (k1_*z_box_order[0, 0]-z_box_order[0, 1]- \
                k2_*z_box_order[1, 0]+z_box_order[1, 1])/(k1_-k2_)
            z_center[1] = k1_*(z_center[0]-z_box_order[0, 0])+z_box_order[0, 1]
        z_center[0] = int(z_center[0])
        z_center[1] = int(z_center[1])

        # 获得砖块中心与3个角点像素坐标，永远计算砖块四元数
        z_box_small = [z_center]
        small_set = 0.1
        for i in range(0, len(z_box_order)-1):
            x_ = int(z_box_order[i, 0]+small_set*(z_center[0]-z_box_order[i, 0]))
            y_ = int(z_box_order[i, 1]+small_set*(z_center[1]-z_box_order[i, 1]))
            z_box_small.append([x_, y_])


        im_temp = img.copy()
        for j in range(len(z_box_small)):
            cv2.circle(im_temp, tuple(z_box_small[j]), 3, (255, 0, 0))  # 画圆
            cv2.putText(im_temp, str(j), tuple(z_box_small[j]), 1, 2, (0,255,0), 2)

        im_temp = cv2.drawContours(im_temp, [cnt_used], -1, (0, 0, 255), \
            thickness=1)  # 画出轮廓图
        rect_temp = np.array(z_rect)
        rect_temp = rect_temp.reshape((len(rect_temp), 1, 2))
        im_temp = cv2.drawContours(im_temp, [rect_temp], -1, (255, 0, 0), \
            thickness=1)  # 画出轮廓图
        cv2.line(im_temp,tuple(z_box_order[0]),tuple(z_box_order[2]),(0,255,0),1)
        cv2.line(im_temp,tuple(z_box_order[1]),tuple(z_box_order[3]),(0,255,0),1)
        cv2.circle(im_temp, tuple(z_center), 3, (255, 0, 0))  # 画圆
        cv2.imshow("im_temp window", im_temp)
        # cv2.imshow("hsv window", hsv)
        cv2.waitKey(3)

        position_ = z_box_small
        return position_

    def brick_select(self):
        pass

    def depth2world(self, point, depth, c_para):
        """parameter:
        point: n*(x, y) 像素坐标
        depth: n*d 深度值
        c_para： (fx, fy, cx, cy) 相机内参
        """
        # 获取输入点的相机世界坐标
        world_p = []
        for i in range(0, len(point)):
            p_ = point[i]
            deep_ = depth[i]
            world_ = []
            if np.isnan(deep_):
                print("depth error: ", point[i], deep_)
                return ERR_POS
            world_.append(deep_*(p_[0]-c_para[2])/c_para[0])
            world_.append(deep_*(p_[1]-c_para[3])/c_para[1])
            world_.append(deep_)
            world_.append(1.0)
            world_p.append(world_)

        # 获取砖块的相机世界坐标的四元数
        p_ = [world_p[i][0:3] for i in range(1, 4)]
        p_ = np.linalg.inv(np.array(p_))
        r3_ = np.dot(p_, np.array([1.0, 1.0, 1.0]).T)
        r3_ = r3_/np.linalg.norm(r3_, axis=0)
        r1_ = np.array([(world_p[2][i]-world_p[1][i]) \
            for i in range(0, len(world_p[1])-1)])
        r1_ = r1_/np.linalg.norm(r1_, axis=0)
        r2_ = np.zeros_like(r1_)
        r2_[0] = r3_[1]*r1_[2]-r3_[2]*r1_[1]
        r2_[1] = r3_[2]*r1_[0]-r3_[0]*r1_[2]
        r2_[2] = r3_[0]*r1_[1]-r3_[1]*r1_[0]

        world_mat = []
        for i in range(0, len(r1_)):
            world_mat.append([r1_[i], r2_[i], r3_[i], world_p[0][i]])
        world_mat.append([0.0, 0.0, 0.0, 1.0])

        return world_mat

    def brick2base(self, brick2camera_mat):
        brick2base = [0]*8
        if brick2camera_mat == ERR_POS:
            return brick2base

        q=tf.transformations.quaternion_from_matrix(brick2camera_mat)
        brick_camera = PoseStamped()
        brick_camera.header.stamp = rospy.Time(0);
        brick_camera.header.frame_id = "kinect_frame_optical";
        brick_camera.pose.position.x = brick2camera_mat[0][3];
        brick_camera.pose.position.y = brick2camera_mat[1][3];
        brick_camera.pose.position.z = brick2camera_mat[2][3];
        brick_camera.pose.orientation.x = q[0];
        brick_camera.pose.orientation.y = q[1];
        brick_camera.pose.orientation.z = q[2];    
        brick_camera.pose.orientation.w = q[3];

        try:
            #(trans,rot) =self.tflistener.lookupTransform('/base_link', '/kinect_frame_optical', rospy.Time(0))
            brick_base=self.tflistener.transformPose("base_link",brick_camera);
            #rospy.loginfo(brick_base)
            brick2base[0] = 1;
            brick2base[1] = brick_base.pose.position.x;
            brick2base[2] = brick_base.pose.position.y;
            brick2base[3] = brick_base.pose.position.z;
            brick2base[4] = brick_base.pose.orientation.x;
            brick2base[5] = brick_base.pose.orientation.y;
            brick2base[6] = brick_base.pose.orientation.z;
            brick2base[7] = brick_base.pose.orientation.w;
            return brick2base
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("compute the brick2base failed")
            return False
 

def main(args):
    rospy.init_node("brick_detection", anonymous=True)
    ic = brick_detection()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 

if __name__ == '__main__':
    main(sys.argv)
