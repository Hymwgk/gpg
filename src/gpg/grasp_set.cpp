#include <gpg/grasp_set.h>


const int GraspSet::ROTATION_AXIS_NORMAL = 0;
const int GraspSet::ROTATION_AXIS_BINORMAL = 1;
const int GraspSet::ROTATION_AXIS_CURVATURE_AXIS = 2;

const bool GraspSet::MEASURE_TIME = false;

const double MAX = (double) (((214013)>>16)&0x7FFF);

int GraspSet::seed_ = 0;


GraspSet::GraspSet() : rotation_axis_(-1)
{
  sample_.setZero();
  hands_.resize(0);
  is_valid_.resize(0);
  angles_.resize(0);
  inner_points_.resize(0);
}


GraspSet::GraspSet(const HandGeometry& hand_geometry, const Eigen::VectorXd& angles, int rotation_axis)
: hand_geometry_(hand_geometry), angles_(angles), rotation_axis_(rotation_axis)
{
  sample_.setZero();
  hands_.resize(0);
  is_valid_.resize(0);
  inner_points_.resize(0);
}

void GraspSet::plotFrameAxes(const Eigen::Matrix4d& frame, const Eigen::Matrix3Xd& points) const
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Frame Axes"));  
  viewer->setBackgroundColor(0.0, 0.0, 0.0);
  viewer->setPosition(0, 0);
  viewer->setSize(640, 480);

  pcl::visualization::Camera camera;
  camera.clip[0] = 0.00130783;
  camera.clip[1] = 1.30783;
  camera.focal[0] = 0.776838;
  camera.focal[1] = -0.095644;
  camera.focal[2] = -0.18991;
  camera.pos[0] = 0.439149;
  camera.pos[1] = -0.10342;
  camera.pos[2] = 0.111626;
  camera.view[0] = 0.666149;
  camera.view[1] = -0.0276846;
  camera.view[2] = 0.745305;
  camera.fovy = 0.8575;
  camera.window_pos[0] = 0;
  camera.window_pos[1] = 0;
  camera.window_size[0] = 640;
  camera.window_size[1] = 480;
  viewer->setCameraParameters(camera);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>  cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<points.cols();i++)
  {
    pcl::PointXYZ temp;
    temp.x = points(0,i);
    temp.y = points(1,i);
    temp.z = points(2,i);
    cloud->push_back(temp);
    }


  viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
    "registered point cloud");

  pcl::PointXYZ o, x_, y_,z_;
  o.x = frame(0,3);
  o.y = frame(1,3);
  o.z = frame(2,3);
  x_.x = o.x + 0.02 * frame(0,0);
  x_.y = o.y + 0.02 * frame(1,0);
  x_.z = o.z + 0.02 * frame(2,0);

  y_.x = o.x + 0.02 * frame(0,1);
  y_.y = o.y + 0.02 * frame(1,1);
  y_.z = o.z + 0.02 * frame(2,1);

  z_.x = o.x + 0.02 * frame(0,2);
  z_.y = o.y + 0.02 * frame(1,2);
  z_.z = o.z + 0.02 * frame(2,2);


  viewer->addLine<pcl::PointXYZ>(o, x_, 255, 0, 0, "x_axis_");
  viewer->addLine<pcl::PointXYZ>(o, y_, 0, 255, 0, "y_axis_" );
  viewer->addLine<pcl::PointXYZ>(o, z_, 0, 0, 255, "z_axis_");


  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  viewer->close();
}

void GraspSet::plotFramePlane(const Eigen::Matrix4d& frame, const Eigen::Matrix3Xd& points) const
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Frame Axes"));  
  viewer->setBackgroundColor(0.0, 0.0, 0.0);
  viewer->setPosition(0, 0);
  viewer->setSize(640, 480);

  pcl::visualization::Camera camera;
  camera.clip[0] = 0.00130783;
  camera.clip[1] = 1.30783;
  camera.focal[0] = 0.776838;
  camera.focal[1] = -0.095644;
  camera.focal[2] = -0.18991;
  camera.pos[0] = 0.439149;
  camera.pos[1] = -0.10342;
  camera.pos[2] = 0.111626;
  camera.view[0] = 0.666149;
  camera.view[1] = -0.0276846;
  camera.view[2] = 0.745305;
  camera.fovy = 0.8575;
  camera.window_pos[0] = 0;
  camera.window_pos[1] = 0;
  camera.window_size[0] = 640;
  camera.window_size[1] = 480;
  viewer->setCameraParameters(camera);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>  cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<points.cols();i++)
  {
    pcl::PointXYZ temp;
    temp.x = points(0,i);
    temp.y = points(1,i);
    temp.z = points(2,i);
    cloud->push_back(temp);
    }


  viewer->addPointCloud<pcl::PointXYZ>(cloud, "registered point cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
    "registered point cloud");

  //根据坐标系位姿，反解出x-o-y平面参数
  double d = -(frame(0,2)*frame(0,3)+frame(1,2)*frame(1,3)+frame(2,2)*frame(2,3));
  pcl::ModelCoefficients coeffs;
  std::cout << "table coefficients: " << float(frame(0,2)) << " " 
                                        << float(frame(1,2)) << " "
                                        << float(frame(2,2)) << " " 
                                        << float(d) << std::endl;

  coeffs.values.push_back (float(frame(0,2)));
  coeffs.values.push_back (float(frame(1,2)));
  coeffs.values.push_back (float(frame(2,2)));
  coeffs.values.push_back (float(d));
  viewer->addPlane (coeffs);

  pcl::PointXYZ o, x_, y_,z_;
  o.x = frame(0,3);
  o.y = frame(1,3);
  o.z = frame(2,3);
  x_.x = o.x + 0.02 * frame(0,0);
  x_.y = o.y + 0.02 * frame(1,0);
  x_.z = o.z + 0.02 * frame(2,0);

  y_.x = o.x + 0.02 * frame(0,1);
  y_.y = o.y + 0.02 * frame(1,1);
  y_.z = o.z + 0.02 * frame(2,1);

  z_.x = o.x + 0.02 * frame(0,2);
  z_.y = o.y + 0.02 * frame(1,2);
  z_.z = o.z + 0.02 * frame(2,2);


  viewer->addLine<pcl::PointXYZ>(o, x_, 255, 0, 0, "x_axis_");
  viewer->addLine<pcl::PointXYZ>(o, y_, 0, 255, 0, "y_axis_" );
  viewer->addLine<pcl::PointXYZ>(o, z_, 0, 0, 255, "z_axis_");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  viewer->close();
}


void GraspSet::plotFramePlane(const Eigen::Matrix4d& frame, 
    const Eigen::Matrix3Xd& partal_points,const Eigen::Matrix3Xd& origin_points) const
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Frame Axes"));  
  viewer->setBackgroundColor(0.0, 0.0, 0.0);
  viewer->setPosition(0, 0);
  viewer->setSize(640, 480);

  pcl::visualization::Camera camera;
  camera.clip[0] = 0.00130783;
  camera.clip[1] = 1.30783;
  camera.focal[0] = 0.776838;
  camera.focal[1] = -0.095644;
  camera.focal[2] = -0.18991;
  camera.pos[0] = 0.439149;
  camera.pos[1] = -0.10342;
  camera.pos[2] = 0.111626;
  camera.view[0] = 0.666149;
  camera.view[1] = -0.0276846;
  camera.view[2] = 0.745305;
  camera.fovy = 0.8575;
  camera.window_pos[0] = 0;
  camera.window_pos[1] = 0;
  camera.window_size[0] = 640;
  camera.window_size[1] = 480;
  viewer->setCameraParameters(camera);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>  cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<partal_points.cols();i++)
  {
    pcl::PointXYZ temp;
    temp.x = partal_points(0,i);
    temp.y = partal_points(1,i);
    temp.z = partal_points(2,i);
    cloud_1->push_back(temp);
    }

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>  cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=0;i<origin_points.cols();i++)
  {
    pcl::PointXYZ temp;
    temp.x = origin_points(0,i);
    temp.y = origin_points(1,i);
    temp.z = origin_points(2,i);
    cloud_2->push_back(temp);
    }

  viewer->addPointCloud<pcl::PointXYZ>(cloud_2, "origin point cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
    "origin point cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_1, 0, 0,255); 
  viewer->addPointCloud<pcl::PointXYZ>(cloud_1,single_color, "partial point cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
    "partial point cloud");

  //根据坐标系位姿，反解出x-o-y平面参数
  double d = -(frame(0,2)*frame(0,3)+frame(1,2)*frame(1,3)+frame(2,2)*frame(2,3));
  pcl::ModelCoefficients coeffs;
  std::cout << "table coefficients: " << float(frame(0,2)) << " " 
                                        << float(frame(1,2)) << " "
                                        << float(frame(2,2)) << " " 
                                        << float(d) << std::endl;

  coeffs.values.push_back (float(frame(0,2)));
  coeffs.values.push_back (float(frame(1,2)));
  coeffs.values.push_back (float(frame(2,2)));
  coeffs.values.push_back (float(d));
  viewer->addPlane (coeffs);

  pcl::PointXYZ o, x_, y_,z_;
  o.x = frame(0,3);
  o.y = frame(1,3);
  o.z = frame(2,3);
  x_.x = o.x + 0.02 * frame(0,0);
  x_.y = o.y + 0.02 * frame(1,0);
  x_.z = o.z + 0.02 * frame(2,0);

  y_.x = o.x + 0.02 * frame(0,1);
  y_.y = o.y + 0.02 * frame(1,1);
  y_.z = o.z + 0.02 * frame(2,1);

  z_.x = o.x + 0.02 * frame(0,2);
  z_.y = o.y + 0.02 * frame(1,2);
  z_.z = o.z + 0.02 * frame(2,2);


  viewer->addLine<pcl::PointXYZ>(o, x_, 255, 0, 0, "x_axis_");
  viewer->addLine<pcl::PointXYZ>(o, y_, 0, 255, 0, "y_axis_" );
  viewer->addLine<pcl::PointXYZ>(o, z_, 0, 0, 255, "z_axis_");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  viewer->close();
}


void GraspSet::evaluateHypotheses(const PointList& point_list, 
      const LocalFrame& local_frame,const Eigen::Matrix4d& table_pose_local)
//对单个局部坐标系进行评估
{
  hands_.resize(angles_.size());
  inner_points_.resize(angles_.size());
  // Local reference frame 之前得到的一个虚拟典范夹爪坐标系
  Eigen::Matrix3d local_frame_mat;
  local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();

  Eigen::MatrixXd local_frame_=Eigen::MatrixXd::Identity(4,4);
  local_frame_.block(0,0,3,3)=local_frame_mat;
  local_frame_.block(0,3,3,1)=local_frame.getSample();


  
  //plotFramePlane(table_pose_local,point_list.getPoints(),origin_point_list.getPoints());
  sample_ = local_frame.getSample();
  is_valid_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, angles_.size(), false);
  //
  FingerHand finger_hand(hand_geometry_.finger_width_, hand_geometry_.outer_diameter_, hand_geometry_.depth_,hand_geometry_.height_);
  Eigen::Matrix3d rot_binormal;

  // Set the lateral and forward axis of the robot hand frame (closing direction and grasp approach direction).
  if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
  {
    finger_hand.setLateralAxis(1);
    finger_hand.setForwardAxis(0);

    // Rotation about binormal by 180 degrees (reverses direction of normal)
    rot_binormal <<  -1.0,  0.0,  0.0,
      0.0,  1.0,  0.0,
      0.0,  0.0, -1.0;
  }
  //plotFramePlane(table_pose_local,point_list.getPoints());



  //int num=0;
  // Evaluate grasp at each hand orientation.
  //每个抓取局部坐标系都有几个旋转值，每个旋转值对应一个候选抓取
  for (int i = 0; i < angles_.rows(); i++)
  {
    // Rotation about curvature axis by <angles_(i)> radians
    Eigen::Matrix3d rot;
    rot <<  cos(angles_(i)),  -1.0 * sin(angles_(i)),   0.0,
      sin(angles_(i)),  cos(angles_(i)),          0.0,
      0.0,              0.0,                      1.0;

    // Rotate points into this hand orientation. 把点云按照夹爪姿态旋转到局部坐标系中
    Eigen::Matrix3d frame_rot;
    frame_rot.noalias() = local_frame_mat * rot_binormal * rot;
    //仅仅对点云进行旋转，不平移，因为之前已经平移过了
    PointList point_list_frame = point_list.rotatePointList(frame_rot.transpose());
    //对桌面坐标系进行旋转
    Eigen::Matrix4d table_pose_frame = table_pose_local;
    table_pose_frame.block(0,0,3,3) = frame_rot.transpose()*table_pose_frame.block(0,0,3,3);//姿态部分
    table_pose_frame.block(0,3,3,1) = frame_rot.transpose()*table_pose_frame.block(0,3,3,1);//位置部分
    //plotFramePlane(table_pose_local,point_list.getPoints(),origin_point_list.getPoints());
    //plotFramePlane(table_pose_frame,point_list_frame.getPoints(),origin_point_list_frame.getPoints());
    //plotFramePlane(table_pose_frame,point_list_frame.getPoints());

    // Crop points on hand height.将点云切片，只要夹爪高度区域内部的一块点云
    PointList point_list_cropped = point_list_frame.cropByHandHeight(hand_geometry_.height_);

    // Evaluate finger placements for this orientation.评估在binormal轴向不同的偏移量下，
    //评估左右手指是否与点云接触
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand_geometry_.init_bite_);

    // Check that there is at least one feasible 2-finger placement.
    //如果某个偏移量下，对应的夹爪左右指都没有碰撞，就说明当前偏移量下，夹爪可行
    //评价出至少存在1个发生碰撞的抓取偏移量
    finger_hand.evaluateHand();

    //检查夹爪是否与桌面碰撞
    //finger_hand.collisionCheckHandTable(table_pose_frame);


    if (finger_hand.getHand().any())
    {
      // Try to move the hand as deep as possible onto the object.尽可能使夹爪在approach方向更加靠近物体点云表面
      //得到的finger_idx 也就是最好的一个
      int finger_idx = finger_hand.deepenHand(point_list_cropped.getPoints(), hand_geometry_.init_bite_, hand_geometry_.depth_);

      // Calculate points in the closing region of the hand.如果夹爪内部没有点，就舍弃该抓取姿态
      std::vector<int> indices_closing = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints(), finger_idx);
      //夹爪内部至少要有一个点
      if (indices_closing.size() == 0)
      {
        continue;
      }
      //检查夹爪是否与桌面碰撞
      if (finger_hand.collisionCheckHandTable(table_pose_frame))
      {
        continue;
      }

      //把夹爪内部的点抽取出来
      PointList points_in_closing_region=point_list_cropped.slice(indices_closing);

      // create the grasp hypothesis针对该旋转angle创建一个候选抓取配置
      Grasp hand = createHypothesis(local_frame.getSample(), point_list_cropped, indices_closing, frame_rot,
        finger_hand);
      //将夹爪各个点旋转到桌面坐标系下


      //把当前候选抓取配置存起来
      hands_[i] = hand;
      is_valid_[i] = true;
      //num++;
      //在这里把截取的点云区域存下来
      inner_points_[i]=points_in_closing_region;
    }
  }
  //std::cout<< num<<std::endl;
}



void GraspSet::evaluateHypotheses(const PointList& point_list, const PointList& origin_point_list, 
    const LocalFrame& local_frame,const Eigen::Matrix4d& table_pose_local)
//对单个局部坐标系进行评估
{
  hands_.resize(angles_.size());
  inner_points_.resize(angles_.size());
  // Local reference frame 之前得到的一个虚拟典范夹爪坐标系
  Eigen::Matrix3d local_frame_mat;
  local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();

  Eigen::MatrixXd local_frame_=Eigen::MatrixXd::Identity(4,4);
  local_frame_.block(0,0,3,3)=local_frame_mat;
  local_frame_.block(0,3,3,1)=local_frame.getSample();


  
  //plotFramePlane(table_pose_local,point_list.getPoints(),origin_point_list.getPoints());
  sample_ = local_frame.getSample();
  is_valid_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, angles_.size(), false);
  //
  FingerHand finger_hand(hand_geometry_.finger_width_, hand_geometry_.outer_diameter_, hand_geometry_.depth_,hand_geometry_.height_);
  Eigen::Matrix3d rot_binormal;

  // Set the lateral and forward axis of the robot hand frame (closing direction and grasp approach direction).
  if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
  {
    finger_hand.setLateralAxis(1);
    finger_hand.setForwardAxis(0);

    // Rotation about binormal by 180 degrees (reverses direction of normal)
    rot_binormal <<  -1.0,  0.0,  0.0,
      0.0,  1.0,  0.0,
      0.0,  0.0, -1.0;
  }
  //plotFramePlane(table_pose_local,point_list.getPoints());



  //int num=0;
  // Evaluate grasp at each hand orientation.
  //每个抓取局部坐标系都有几个旋转值，每个旋转值对应一个候选抓取
  for (int i = 0; i < angles_.rows(); i++)
  {
    // Rotation about curvature axis by <angles_(i)> radians
    Eigen::Matrix3d rot;
    rot <<  cos(angles_(i)),  -1.0 * sin(angles_(i)),   0.0,
      sin(angles_(i)),  cos(angles_(i)),          0.0,
      0.0,              0.0,                      1.0;

    // Rotate points into this hand orientation. 把点云按照夹爪姿态旋转到局部坐标系中
    Eigen::Matrix3d frame_rot;
    frame_rot.noalias() = local_frame_mat * rot_binormal * rot;
    //仅仅对点云进行旋转，不平移，因为之前已经平移过了
    PointList point_list_frame = point_list.rotatePointList(frame_rot.transpose());
    PointList origin_point_list_frame = origin_point_list.rotatePointList(frame_rot.transpose());
    //对桌面坐标系进行旋转
    Eigen::Matrix4d table_pose_frame = table_pose_local;
    table_pose_frame.block(0,0,3,3) = frame_rot.transpose()*table_pose_frame.block(0,0,3,3);//姿态部分
    table_pose_frame.block(0,3,3,1) = frame_rot.transpose()*table_pose_frame.block(0,3,3,1);//位置部分
    //plotFramePlane(table_pose_local,point_list.getPoints(),origin_point_list.getPoints());
    //plotFramePlane(table_pose_frame,point_list_frame.getPoints(),origin_point_list_frame.getPoints());
    //plotFramePlane(table_pose_frame,point_list_frame.getPoints());

    // Crop points on hand height.将点云切片，只要夹爪高度区域内部的一块点云
    PointList point_list_cropped = point_list_frame.cropByHandHeight(hand_geometry_.height_);

    // Evaluate finger placements for this orientation.评估在binormal轴向不同的偏移量下，
    //评估左右手指是否与点云接触
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand_geometry_.init_bite_);

    // Check that there is at least one feasible 2-finger placement.
    //如果某个偏移量下，对应的夹爪左右指都没有碰撞，就说明当前偏移量下，夹爪可行
    //评价出至少存在1个发生碰撞的抓取偏移量
    finger_hand.evaluateHand();

    //检查夹爪是否与桌面碰撞
    //finger_hand.collisionCheckHandTable(table_pose_frame);


    if (finger_hand.getHand().any())
    {
      // Try to move the hand as deep as possible onto the object.尽可能使夹爪在approach方向更加靠近物体点云表面
      //得到的finger_idx 也就是最好的一个
      int finger_idx = finger_hand.deepenHand(point_list_cropped.getPoints(), hand_geometry_.init_bite_, hand_geometry_.depth_);

      // Calculate points in the closing region of the hand.如果夹爪内部没有点，就舍弃该抓取姿态
      std::vector<int> indices_closing = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints(), finger_idx);
      //夹爪内部至少要有一个点
      if (indices_closing.size() == 0)
      {
        continue;
      }
      //检查夹爪是否与桌面碰撞
      if (finger_hand.collisionCheckHandTable(table_pose_frame))
      {
        continue;
      }

      //把夹爪内部的点抽取出来
      PointList points_in_closing_region=point_list_cropped.slice(indices_closing);

      // create the grasp hypothesis针对该旋转angle创建一个候选抓取配置
      Grasp hand = createHypothesis(local_frame.getSample(), point_list_cropped, indices_closing, frame_rot,
        finger_hand);
      //将夹爪各个点旋转到桌面坐标系下


      //把当前候选抓取配置存起来
      hands_[i] = hand;
      is_valid_[i] = true;
      //num++;
      //在这里把截取的点云区域存下来
      inner_points_[i]=points_in_closing_region;

    }
  }
  //std::cout<< num<<std::endl;
}

void GraspSet::evaluateHypotheses(const PointList& point_list, const LocalFrame& local_frame)
//对单个局部坐标系进行评估
{
  hands_.resize(angles_.size());
  inner_points_.resize(angles_.size());

  sample_ = local_frame.getSample();
  is_valid_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, angles_.size(), false);
  //
  FingerHand finger_hand(hand_geometry_.finger_width_, hand_geometry_.outer_diameter_, hand_geometry_.depth_,hand_geometry_.height_);
  Eigen::Matrix3d rot_binormal;

  // Set the lateral and forward axis of the robot hand frame (closing direction and grasp approach direction).
  if (rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
  {
    finger_hand.setLateralAxis(1);
    finger_hand.setForwardAxis(0);

    // Rotation about binormal by 180 degrees (reverses direction of normal)
    //保证初始状态时，夹爪approach方向与表面法向量方向相反
    rot_binormal <<  -1.0,  0.0,  0.0,
                                          0.0,  1.0,  0.0,
                                          0.0,  0.0, -1.0;
  }

  // Local reference frame 局部坐标系在相机坐标系中的位置姿态
  Eigen::Matrix3d local_frame_mat;
  local_frame_mat << local_frame.getNormal(), local_frame.getBinormal(), local_frame.getCurvatureAxis();

  // Evaluate grasp at each hand orientation.
  //每个抓取局部坐标系都有几个旋转值，每个旋转值对应一个候选抓取
  for (int i = 0; i < angles_.rows(); i++)
  {
    // Rotation about curvature axis by <angles_(i)> radians
    Eigen::Matrix3d rot;
    //这里表现的是绕运动坐标系z轴旋转
    rot <<  cos(angles_(i)),  -1.0 * sin(angles_(i)),   0.0,
      sin(angles_(i)),  cos(angles_(i)),          0.0,
      0.0,              0.0,                      1.0;

    // Rotate points into this hand orientation. 把点云按照夹爪姿态旋转到局部坐标系中
    Eigen::Matrix3d frame_rot;
    //右乘，相对于local frame做变换，得到新的相对于相机坐标系的坐标系表示
    frame_rot.noalias() = local_frame_mat * rot_binormal * rot;
    //仅仅对点云进行旋转，不平移，因为之前已经平移过了
    PointList point_list_frame = point_list.rotatePointList(frame_rot.transpose());

    // Crop points on hand height.将点云切片，只要夹爪高度区域内部的一块点云
    PointList point_list_cropped = point_list_frame.cropByHandHeight(hand_geometry_.height_);

    // Evaluate finger placements for this orientation.评估在binormal轴向不同的偏移量下，
    //评估左右手指是否与点云接触
    finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand_geometry_.init_bite_);

    // Check that there is at least one feasible 2-finger placement.
    //如果某个偏移量下，对应的夹爪左右指都没有碰撞，就说明当前偏移量下，夹爪可行
    //评价出至少存在1个发生碰撞的抓取偏移量
    finger_hand.evaluateHand();

    if (finger_hand.getHand().any())
    {
      // Try to move the hand as deep as possible onto the object.尽可能使夹爪更加靠近物体点云表面
      int finger_idx = finger_hand.deepenHand(point_list_cropped.getPoints(), hand_geometry_.init_bite_, hand_geometry_.depth_);

      // Calculate points in the closing region of the hand.如果夹爪内部没有点，就舍弃该抓取姿态
      std::vector<int> indices_closing = finger_hand.computePointsInClosingRegion(point_list_cropped.getPoints(), finger_idx);
      //夹爪内部至少要有一个点
      if (indices_closing.size() == 0)
      {
        continue;
      }
      //把夹爪内部的点抽取出来
      PointList points_in_closing_region=point_list_cropped.slice(indices_closing);

      // create the grasp hypothesis针对该旋转angle创建一个候选抓取配置
      Grasp hand = createHypothesis(local_frame.getSample(), point_list_cropped, indices_closing, frame_rot,
        finger_hand);
      //将夹爪各个点旋转到桌面坐标系下


      //把当前候选抓取配置存起来
      hands_[i] = hand;
      is_valid_[i] = true;
      //在这里把截取的点云区域存下来
      inner_points_[i]=points_in_closing_region;

    }
  }
}


Eigen::Matrix3Xd GraspSet::calculateShadow4(const PointList& point_list, double shadow_length) const
{
  double voxel_grid_size = 0.003; // voxel size for points that fill occluded region

  double num_shadow_points = floor(shadow_length / voxel_grid_size); // number of points along each shadow vector

  const int num_cams = point_list.getCamSource().rows();

  // Calculate the set of cameras which see the points.
  Eigen::VectorXi camera_set = point_list.getCamSource().rowwise().sum();

  // Calculate the center point of the point neighborhood.
  Eigen::Vector3d center = point_list.getPoints().rowwise().sum();
  center /= (double) point_list.size();

  // Stores the list of all bins of the voxelized, occluded points.
  std::vector< Vector3iSet > shadows;
  shadows.resize(num_cams, Vector3iSet(num_shadow_points * 10000));

  for (int i = 0; i < num_cams; i++)
  {
    if (camera_set(i) >= 1)
    {
      double t0_if = omp_get_wtime();

      // Calculate the unit vector that points from the camera position to the center of the point neighborhood.
      Eigen::Vector3d shadow_vec = center - point_list.getViewPoints().col(i);

      // Scale that vector by the shadow length.
      shadow_vec = shadow_length * shadow_vec / shadow_vec.norm();

      // Calculate occluded points.
      //      shadows[i] = calculateVoxelizedShadowVectorized4(point_list, shadow_vec, num_shadow_points, voxel_grid_size);
      calculateVoxelizedShadowVectorized(point_list.getPoints(), shadow_vec, num_shadow_points, voxel_grid_size, shadows[i]);
    }
  }

  // Only one camera view point.
  if (num_cams == 1)
  {
    // Convert voxels back to points.
    //    std::vector<Eigen::Vector3i> voxels(shadows[0].begin(), shadows[0].end());
    //    Eigen::Matrix3Xd shadow_out = shadowVoxelsToPoints(voxels, voxel_grid_size);
    //    return shadow_out;
    return shadowVoxelsToPoints(std::vector<Eigen::Vector3i>(shadows[0].begin(), shadows[0].end()), voxel_grid_size);
  }

  // Multiple camera view points: find the intersection of all sets of occluded points.
  double t0_intersection = omp_get_wtime();
  Vector3iSet bins_all = shadows[0];

  for (int i = 1; i < num_cams; i++)
  {
    if (camera_set(i) >= 1) // check that there are points seen by this camera
    {
      bins_all = intersection(bins_all, shadows[i]);
    }
  }
  if (MEASURE_TIME)
    std::cout << "intersection runtime: " << omp_get_wtime() - t0_intersection << "s\n";

  // Convert voxels back to points.
  std::vector<Eigen::Vector3i> voxels(bins_all.begin(), bins_all.end());
  Eigen::Matrix3Xd shadow_out = shadowVoxelsToPoints(voxels, voxel_grid_size);
  return shadow_out;
}


Eigen::Matrix3Xd GraspSet::shadowVoxelsToPoints(const std::vector<Eigen::Vector3i>& voxels, double voxel_grid_size) const
{
  // Convert voxels back to points.
  double t0_voxels = omp_get_wtime();
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);

  Eigen::Matrix3Xd shadow(3, voxels.size());

  for (int i = 0; i < voxels.size(); i++)
  {
    shadow.col(i) = voxels[i].cast<double>() * voxel_grid_size + Eigen::Vector3d::Ones() * generator()
        * voxel_grid_size * 0.3;
    //    shadow.col(i) = voxels[i].cast<double>() * voxel_grid_size;
    //    shadow.col(i)(0) += generator() * voxel_grid_size * 0.3;
    //    shadow.col(i)(1) += generator() * voxel_grid_size * 0.3;
    //    shadow.col(i)(2) += generator() * voxel_grid_size * 0.3;
  }
  if (MEASURE_TIME)
    std::cout << "voxels-to-points runtime: " << omp_get_wtime() - t0_voxels << "s\n";

  return shadow;
}


void GraspSet::calculateVoxelizedShadowVectorized(const Eigen::Matrix3Xd& points,
  const Eigen::Vector3d& shadow_vec, int num_shadow_points, double voxel_grid_size, Vector3iSet& shadow_set) const
{
  double t0_set = omp_get_wtime();
  const int n = points.cols() * num_shadow_points;
  const double voxel_grid_size_mult = 1.0 / voxel_grid_size;
  const double max = 1.0 / 32767.0;
  //  Eigen::Vector3d w;

  for(int i = 0; i < n; i++)
  {
    const int pt_idx = i / num_shadow_points;
    //    const Eigen::Vector3d w = (points.col(pt_idx) + ((double) fastrand() * max) * shadow_vec) * voxel_grid_size_mult;
    shadow_set.insert(((points.col(pt_idx) + ((double) fastrand() * max) * shadow_vec) * voxel_grid_size_mult).cast<int>());
  }

  if (MEASURE_TIME)
    printf("Shadow (1 camera) calculation. Runtime: %.3f, #points: %d, num_shadow_points: %d, #shadow: %d, max #shadow: %d\n",
      omp_get_wtime() - t0_set, (int) points.cols(), num_shadow_points, (int) shadow_set.size(), n);
  //    std::cout << "Calculated shadow for 1 camera. Runtime: " << omp_get_wtime() - t0_set << ", #points: " << n << "\n";
}


Grasp GraspSet::createHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
  const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const
{
  // extract data for classification
  PointList point_list_learning = point_list.slice(indices_learning);

//  std::cout << point_list_learning.getPoints().block(0,0,3,5) << "\n";
//  std::cout << "---------------\n";
//  std::cout << point_list_learning.getNormals().block(0,0,3,5) << "\n";

  // calculate grasp width (hand opening width)
  double width = point_list_learning.getPoints().row(0).maxCoeff() - point_list_learning.getPoints().row(0).minCoeff();

  Grasp hand(sample, hand_frame, finger_hand, width);

  // evaluate if the grasp is antipodal
  labelHypothesis(point_list_learning, finger_hand, hand);

  return hand;
}


void GraspSet::labelHypothesis(const PointList& point_list, const FingerHand& finger_hand, Grasp& hand)
const
{
  Antipodal antipodal;
  int label = antipodal.evaluateGrasp(point_list, 0.003, finger_hand.getLateralAxis(), finger_hand.getForwardAxis(),
    rotation_axis_);
  hand.setHalfAntipodal(label == Antipodal::HALF_GRASP || label == Antipodal::FULL_GRASP);
  hand.setFullAntipodal(label == Antipodal::FULL_GRASP);
}


inline int GraspSet::fastrand() const
{
  seed_ = (214013*seed_+2531011);
  return (seed_>>16)&0x7FFF;
}


Vector3iSet GraspSet::intersection(const Vector3iSet& set1, const Vector3iSet& set2) const
{
  if (set2.size() < set1.size())
  {
    return intersection(set2, set1);
  }

  Vector3iSet set_out(set1.size());

  for (Vector3iSet::const_iterator it = set1.begin(); it != set1.end(); it++)
  {
    if (set2.find(*it) != set2.end())
    {
      set_out.insert(*it);
    }
  }

  return set_out;
}
