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
ERR_POS = [[0 , 0, 0, 0],[0 , 0, 0, 0], [0 , 0, 0, 0], [0 , 0, 0, 0]]
Z_SCALE = 2.1

class brick_detection:
    def __init__(self):
        self.image_pub = rospy.Publisher("brick_position", Floats,queue_size=1)
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", \
            Image, self.callback)
        self.depth_sub = rospy.Subscriber("/kinect/depth/image_raw", \
            Image, self.callback_depth)
        self.depth_flag = False
 
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
        else:
            brick_depth = []
            for i in img_position:
                brick_depth.append(self.depth_img[i[1], i[0]])
            c_para = C_PARAM  # 相机4个内参
            brick_world = self.depth2world(img_position, brick_depth, c_para)
            cv2.circle(cv_image, tuple(img_position[0]), 5, (255, 0, 0))  # 画圆

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

        self.depth_img = depth_image
        self.depth_flag = True

    def brick_detect(self, img):  # 砖块检测算法
        img = img.copy()
        self.test_img = img.copy()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转hsv颜色空间

        z_hsv = np.array([9, 194, 57])  # 砖块hsv值需要标定
        lower = np.array([2, 150, 70])
        upper = np.array([29, 250, 110])
        z_mask = cv2.inRange(hsv, lower, upper)  # 通过颜色提取砖块的二值图

        # 通过轮廓提取去除砖块中的小黑点
        _, contours, hierarchy = cv2.findContours(z_mask, \
            cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        cnt_small = []
        for i in range(hierarchy.shape[1]):
            if hierarchy[0, i, 2]!=0:
                area = cv2.contourArea(contours[i])
                if area<100:
                    cnt_small.append(contours[i])
        z_mask = cv2.drawContours(z_mask, cnt_small, -1, (255), thickness=-1)

        if True:  # 二值图形态学处理
            elem_o=cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
            # z_open = cv2.morphologyEx(z_mask, cv2.MORPH_OPEN, elem_o)
            # elem_c=cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5, 5))
            # z_out = cv2.morphologyEx(z_open, cv2.MORPH_CLOSE, elem_c)
            z_out = cv2.erode(z_mask, elem_o, iterations=4)
        else:
            z_out = z_mask

        # 提取砖块轮廓
        _, contours, hierarchy = cv2.findContours(z_out, cv2.RETR_EXTERNAL, \
            cv2.CHAIN_APPROX_SIMPLE)  # 从二值像提取轮廓, cv2.CHAIN_APPROX_NONE
        if len(contours)==0:
            return 0

        # 选择砖块的决策
        cnt_used, rect_used, cor_used = self.brick_select(contours)
        if cnt_used==[]:
            return 0

        z_rect = rect_used  # 获取最小外接矩形顶点坐标
        z_cor = cor_used  # 获取拟合多边形顶点坐标

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
        use_rect = 0
        for i in range(z_rect.shape[0]):
            d_min = 10
            for j in range(z_box.shape[0]):
                d_ = np.sqrt((z_rect[i, 0]-z_box[j, 0])**2+\
                    (z_rect[i, 1]-z_box[j, 1])**2)
                if d_<d_min:
                    d_min=d_
            if d_min<5:
                use_rect += 1
        z_center = None
        if use_rect>=3:
            z_center = [np.sum(z_rect[:, 0])/z_rect.shape[0], \
                np.sum(z_rect[:, 1])/z_rect.shape[0]]
        
        # 获取砖块有效坐标(中心+3角点)
        z_box_small = self.box_calc(z_box, z_center=z_center)

        self.imshow_(z_box_small, cnt_used, z_rect)

        return z_box_small

    def brick_select(self, cnt):  # 选择砖块的决策
        cnt_temp = []
        cnt_temp_cor = []
        cnt_choose = []
        cnt_rect = []
        cnt_cor = []
        for i in range(len(cnt)):
            cnt_ = cnt[i]

            # 获取拟合多边形顶点坐标, 处理掉形状异常的轮廓区域
            epsilon = 0.01 * cv2.arcLength(cnt_,True)
            corner_ = cv2.approxPolyDP(cnt_, epsilon, True)
            # 处理掉小的轮廓区域，这个区域的大小自己定义
            area = cv2.contourArea(cnt_)
            # print(len(corner_), area)
            if len(corner_)>=4 and area>7000:
                cnt_temp.append(cnt_)
                cnt_temp_cor.append(corner_)
            else:
                continue

            # 获取最小外接矩形顶点坐标
            z_rect = cv2.minAreaRect(cnt_)
            z_rect = cv2.boxPoints(z_rect)
            z_rect = z_rect.astype(np.int32)
            z_0 = np.sqrt(np.sum(np.multiply(\
                z_rect[0]-z_rect[1], z_rect[0]-z_rect[1])))
            z_1 = np.sqrt(np.sum(np.multiply(\
                z_rect[1]-z_rect[2], z_rect[1]-z_rect[2])))
            z_hw = z_0/z_1 if z_0>z_1 else z_1/z_0

            # 获取轮廓矩形面积比
            z_rect_ = z_rect.reshape((-1, 1, 2))
            area_rect = cv2.contourArea(z_rect_)
            # print(z_hw, area, area_rect, area/area_rect)
            if z_hw<0.7*Z_SCALE or z_hw>1.5*Z_SCALE or area/area_rect<0.7:
                continue

            cnt_choose.append(cnt_)
            cnt_rect.append(z_rect)
            cnt_cor.append(corner_)

        if len(cnt_choose)==1:  # 找到一个不连接的砖块
            return cnt_choose[0], cnt_rect[0], cnt_cor[0]
        elif len(cnt_choose)>1:  # 找到多个不连接的砖块
            p_ = [640/2, 480/2]
            d_ = []
            for i in range(len(cnt_rect)):
                rect_ = cnt_rect[i]
                x_ = np.sum(rect_[:, 0])/len(rect_)
                y_ = np.sum(rect_[:, 1])/len(rect_)
                d_.append(np.sqrt((x_-p_[0])**2+(y_-p_[1])**2))
            ind = np.argmin(d_)
            return cnt_choose[ind], cnt_rect[ind], cnt_cor[ind]
        
        # 未找到不连接的砖块
        return [], [], []

    def box_calc(self, z_box, z_center=None):  # 获取砖块有效坐标(中心+3角点)
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
        if (z_temp1[0, 0]**2+z_temp1[0, 1]**2)<(\
            z_temp1[1, 0]**2+z_temp1[1, 1]**2):
            z_temp1 = z_box_order[0].copy()
            for i in range(len(z_box_order)-1):
                z_box_order[i] = z_box_order[i+1]
            z_box_order[3] = z_temp1

        # 计算砖块中心像素坐标
        if z_center==None:
            z_center = [0, 0]
            if (z_box_order[0, 0]-z_box_order[2, 0])==0 and \
                (z_box_order[1, 0]-z_box_order[3, 0])==0:
                return 0
            elif (z_box_order[0, 0]-z_box_order[2, 0])==0:
                z_center[0] = z_box_order[0, 0]
                k_ = (z_box_order[1, 1]-z_box_order[3, 1])/ \
                    (z_box_order[1, 0]-z_box_order[3, 0])
                z_center[1] = k_*(z_center[0]-z_box_order[1, 0])+ \
                    z_box_order[1, 1]
            elif (z_box_order[1, 0]-z_box_order[3, 0])==0:
                z_center[0] = z_box_order[1, 0]
                k_ = (z_box_order[0, 1]-z_box_order[2, 1])/ \
                    (z_box_order[0, 0]-z_box_order[2, 0])
                z_center[1] = k_*(z_center[0]-z_box_order[0, 0])+ \
                    z_box_order[0, 1]
            else:
                k1_ = 1.0*(z_box_order[0, 1]-z_box_order[2, 1])/ \
                    (z_box_order[0, 0]-z_box_order[2, 0])
                k2_ = 1.0*(z_box_order[1, 1]-z_box_order[3, 1])/ \
                    (z_box_order[1, 0]-z_box_order[3, 0])
                z_center[0] = (k1_*z_box_order[0, 0]-z_box_order[0, 1]- \
                    k2_*z_box_order[1, 0]+z_box_order[1, 1])/(k1_-k2_)
                z_center[1] = k1_*(z_center[0]-z_box_order[0, 0])+ \
                    z_box_order[0, 1]

        z_center[0] = int(z_center[0])
        z_center[1] = int(z_center[1])

        # 获得砖块中心与3个角点像素坐标，永远计算砖块四元数
        z_box_small = [z_center]
        small_set = 0.1
        for i in range(0, len(z_box_order)-1):
            x_ = int(z_box_order[i, 0]+small_set*(z_center[0]-z_box_order[i, 0]))
            y_ = int(z_box_order[i, 1]+small_set*(z_center[1]-z_box_order[i, 1]))
            z_box_small.append([x_, y_])

        return z_box_small

    def depth2world(self, point, depth, c_para):  # 坐标和深度数据-->4*4变换矩阵
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

    def imshow_(self, z_box_small, cnt_used, z_rect):  #显示图像结果
        im_temp = self.test_img.copy()
        for j in range(len(z_box_small)):
            cv2.circle(im_temp, tuple(z_box_small[j]), 3, (255, 0, 0))  # 画圆
            cv2.putText(im_temp, str(j), tuple(z_box_small[j]), 1, 2, (0,255,0), 2)

        im_temp = cv2.drawContours(im_temp, [cnt_used], -1, (0, 0, 255), \
            thickness=1)  # 画出轮廓图
        rect_temp = np.array(z_rect)
        rect_temp = rect_temp.reshape((len(rect_temp), 1, 2))
        im_temp = cv2.drawContours(im_temp, [rect_temp], -1, (255, 0, 0), \
            thickness=1)  # 画出轮廓图
        cv2.line(im_temp,tuple(z_box_small[1]),tuple(z_box_small[2]),(0,255,0),1)
        cv2.line(im_temp,tuple(z_box_small[2]),tuple(z_box_small[3]),(0,255,0),1)
        cv2.circle(im_temp, tuple(z_box_small[0]), 3, (255, 0, 0))  # 画圆
        cv2.imshow("img_det window", im_temp)
        cv2.imshow("deep_img window", self.depth_img)
        cv2.waitKey(3)
 

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
