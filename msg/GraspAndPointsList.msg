# 存储一个抓取配置list，以及对应的夹爪内部的点云列表

# The time of acquisition, and the coordinate frame ID.
Header header
# The list of grasp configurations.
gpg/GraspConfig[] grasps

#对应的抓取姿态下，夹爪内部的点云列表
gpg/GripperInnerPoints[] inner_points