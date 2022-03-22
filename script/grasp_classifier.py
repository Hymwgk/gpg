#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : wgk - ZZU
# E-mail     : 
# Description:
# Date       : 03/18/2022 21:05 PM
# File Name  : 

import torch

import rospy
import os
print('Hello,'+os.environ.get('ROS_PACKAGE_PATH')+'!')
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from gpg.srv._pointnet_gpd import pointnet_gpd,pointnet_gpdResponse
import std_msgs
import numpy as np

import os
from pyquaternion import Quaternion
import sys
from os import path
import time
from scipy.stats import mode
import multiprocessing as mp
import argparse
from model.pointnet import PointNetCls

#解析命令行参数
parser = argparse.ArgumentParser(description='Group and mask generation')
parser.add_argument('--cuda',action='store_true', default=True)  #设置同时处理几个场景
parser.add_argument("--load-model", type=str,default="/home/wgk/catkin_ws/src/gpg/data/1v_750_2class_ourway2sample.model")
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument("--loop_num", type=int, default=3)

args,unknow =parser.parse_known_args()



#sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath("__file__")))))
#sys.path.append(os.environ['HOME'] + "/code/PointNetGPD")
minimal_points_send_to_point_net = 20
input_points_num =750
bad_grasps = []

if args.cuda:
    torch.cuda.manual_seed(1)
np.random.seed(int(time.time()))

model = PointNetCls(num_points=input_points_num, input_chann=3, k=2)

data = torch.load(args.load_model, map_location="cpu")
model.load_state_dict(data.module.state_dict())
if args.cuda:
    if args.gpu != -1:
        torch.cuda.set_device(args.gpu)
        model = model.cuda()



def points2numpy(points):
    array= np.empty([1,3],dtype=np.double)
    for i in range(len(points.points)):
        vector = np.array([points.points[i].x,points.points[i].y,points.points[i].z]).reshape(1,3)
        array=np.concatenate((array,vector),axis=0)
    return array


def test_network(model_, inner_points_batch):
    """使用使用pointnet网络
    """
    inner_points_batch=torch.FloatTensor(np.swapaxes(inner_points_batch,1,2))
    if args.cuda:
        inner_points_batch = inner_points_batch.cuda()
    #输出的实际上只是概率
    output, _ = model_(inner_points_batch)  # N*C 
    output = output.softmax(1)

    return output
    #找到概率最高的那个
    #output = output.softmax(1)
    #pred = output.data.max(1, keepdim=True)[1]
    #output = output.cpu()
    #return pred[0], output.data.numpy()



def pointnet(req):

    print("Got request !")
    inner_points_list = req.candidates.inner_points
    grasp_list = req.candidates.grasps
    if len(grasp_list)==0:
        resp = pointnet_gpdResponse()
        resp.best_grasp_id.data = 0
        #time.sleep(10)
        print("No candidate grasps !")
        return resp
    inner_points_batch = np.empty([0,input_points_num,3])
    good_index = []
    for i in range(len(grasp_list)):
        #将点云转换为numpy数据
        inner_points = points2numpy(inner_points_list[i])
        #记录下来夹爪内部点数过少的抓取
        if inner_points.shape[0] < minimal_points_send_to_point_net:
            rospy.loginfo("Mark as bad grasp! Only {} points, should be at least {} points.".format(
                            inner_points.shape[0], minimal_points_send_to_point_net))
            bad_grasps.append(i)
            continue
        #将夹爪内部点数扩充或删减为固定点数
        if inner_points.shape[0] >= input_points_num:
            #随机抽选其中的一些点，保证和要求的点数量是一致的
            points_modify = inner_points[np.random.choice(inner_points.shape[0],
                                                                input_points_num, replace=False)]
        #如果在线采集的夹爪内部点云点数量是小于  指定的点数量
        else:
            #就是补上去一些点
            points_modify = inner_points[np.random.choice(inner_points.shape[0],
                                                                input_points_num, replace=True)]
        #points_modify.reshape(1,input_points_num,-1)

        #将所有夹爪内部的点云组成一个batch，一起处理
        inner_points_batch=np.concatenate((inner_points_batch,points_modify[None,:,:]),axis=0)
        good_index.append(i)
    
    #print(inner_points_batch.shape)

    out_put = torch.zeros([len(grasp_list),2])
    if args.cuda:
        out_put = out_put.cuda()
    for i in range(args.loop_num):
        #循环预测指定轮数,消除网络随机性影响
        out_put_ = test_network(model.eval(), inner_points_batch)
        out_put = out_put+out_put_
    
    #得到最终预测结果
    #out_put = out_put.softmax(1)
    #找到每个抓取的分类结果
    mask = out_put.max(1)[1]
    #在抓取为good的结果中，找最好的
    score = out_put[:,1]*mask
    best_grasp_id = score.max(0)[1]
    #调整抓取index
    best_grasp_id_ = good_index[best_grasp_id]
    
   
    resp = pointnet_gpdResponse()
    resp.best_grasp_id.data = best_grasp_id_
    #time.sleep(10)
    print("Best grasp id is {}".format(best_grasp_id.cpu().data))
    return resp


if __name__ == '__main__':
    #初始化节点，把节点名称写为grasp_tf_broadcaster  抓取发布器，anonymous参数在
    # 为True的时候会在原本节点名字的后面加一串随机数，来保证可以同时开启多个同样的
    # 节点，如果为false的话就只能开一个
    rospy.init_node('grasp_classifier', anonymous=True)
    #发布检测到的抓取，用于抓取
    #pub2 = rospy.Publisher('/detect_grasps/clustered_grasps', GraspConfigList, queue_size=1)
    server = rospy.Service('grasp_classification', pointnet_gpd, pointnet)
    #
    rate = rospy.Rate(10)

    rospy.loginfo("Ready to check grasp")

    rospy.spin()
