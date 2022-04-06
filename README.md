# Grasp Pose Generator (GPG)

* **Author:** Andreas ten Pas (atp@ccs.neu.edu)
* **Version:** 1.0.0
* **Author's website:** [http://www.ccs.neu.edu/home/atp/](http://www.ccs.neu.edu/home/atp/)
* **License:** BSD


## 1) Overview

This package creates grasp candidates for 3D point clouds and can check if they are antipodal using geometric 
conditions. To use the package, you only need PCL and Eigen (see below).

<img src="readme/examples.png" alt="" style="width: 400px;"/>

This package is part of GPD. Please notice that **no** machine learning is included in this part. The package just 
generates 6-DOF grasp poses for a 2-finger grasp.

将`Grasp Pose Generator (GPG)`与`PointNetGPD`结合在一起，在线候选抓取采样使用GPG生成，然后使用PointNet对候选抓取进行打分排序，并执行最终排序最高的抓取。

本仓库做了如下调整修改：

1. 将原来的GPG抓取采样移植到了ROS框架中
2. 添加了桌面的检测剔除（可选），可剔除一些与桌面发生碰撞的抓取
3. 在`gpg_online.cpp`中以ros服务的形式调用了pointNet的打分服务，并最终把最优的抓取以话题形式发布出去
4. 添加了以`Franka Panda`机器人为平台的简单抓取脚本，可对检测出的最优抓取进行实际执行


## 2) 依赖

1. [PCL 1.7 or later](http://pointclouds.org/)
2. [Eigen 3.0 or later](https://eigen.tuxfamily.org)


## 3) 编译

1. Open a terminal and clone the *grasp_candidates_generator* repository into some folder: 

   ```
   $ cd <location_of_your_workspace>
   $ git clone https://github.com/atenpas/gpg.git
   ```

2. Build the project: 

   ```
   $ cd grasp_candidates_generator
   $ mkdir build && cd build
   $ cmake ..
   $ make
   ```

3. (optional) Install the project:
   ```
   $ sudo make install
   ```


## 4) Generate Grasp Candidates for a Point Cloud File

Run the following from within the *build* folder:

   ```
   $ ./generate_candidates ../cfg/params.cfg ~/data/some_cloud.pcd
   ```


## 5) Parameters

Brief explanations of parameters are given in *cfg/params.cfg*.


## 6) Citation

If you like this package and use it in your own work, please cite our [arXiv paper](http://arxiv.org/abs/1603.01564):

```
@misc{1603.01564,
Author = {Marcus Gualtieri and Andreas ten Pas and Kate Saenko and Robert Platt},
Title = {High precision grasp pose detection in dense clutter},
Year = {2016},
Eprint = {arXiv:1603.01564},
} 
```
