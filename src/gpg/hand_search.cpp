#include <gpg/hand_search.h>


const int HandSearch::ROTATION_AXIS_NORMAL = 0;
const int HandSearch::ROTATION_AXIS_BINORMAL = 1;
const int HandSearch::ROTATION_AXIS_CURVATURE_AXIS = 2;


HandSearch::HandSearch(Parameters params) : params_(params), plots_samples_(false), plots_local_axes_(false),
  plots_camera_sources_(false)
{
  // Calculate radius for nearest neighbor search.
  Eigen::Vector3d hand_dims;
  hand_dims << params_.hand_outer_diameter_ - params_.finger_width_, params_.hand_depth_, params_.hand_height_ / 2.0;
  nn_radius_ = hand_dims.maxCoeff();
}


std::vector<GraspSet> HandSearch::searchHands(const CloudCamera& cloud_cam, bool plots_normals,
  bool plots_samples) const
{
  if (params_.rotation_axis_ < 0 || params_.rotation_axis_ > 2)
  {
    std::cout << "Parameter <rotation_axis> is not set correctly!\n";
    std::vector<GraspSet> empty(0);
    return empty;
  }

  double t0_total = omp_get_wtime();

  // Create KdTree for neighborhood search.
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  // Plot normals and/or samples if desired.
  if (plots_normals)
  {
    std::cout << "Plotting normals ...\n";
    plot_.plotNormals(cloud_cam.getNormals(), cloud_normals_);
  }

  if (plots_samples)
  {
    if (cloud_cam.getSampleIndices().size() > 0)
    {
      std::cout << "Plotting sample indices ...\n";
      plot_.plotSamples(cloud_cam.getSampleIndices(), cloud_cam.getCloudProcessed());
    }
    else if (cloud_cam.getSamples().cols() > 0)
    {
      std::cout << "Plotting samples ...\n";
      plot_.plotSamples(cloud_cam.getSamples(), cloud_cam.getCloudProcessed());
    }
  }

  // 1. Estimate local reference frames.
  //第一步在目标点云中采样得到指定数量的局部坐标系
  std::cout << "Estimating local reference frames ...\n";
  std::vector<LocalFrame> frames;
  FrameEstimator frame_estimator(params_.num_threads_);
  if (cloud_cam.getSamples().cols() > 0)
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSamples(), params_.nn_radius_frames_, kdtree);
  else if (cloud_cam.getSampleIndices().size() > 0)
    frames = frame_estimator.calculateLocalFrames(cloud_cam, cloud_cam.getSampleIndices(), params_.nn_radius_frames_, kdtree);
  else
  {
    std::cout << "Error: No samples or no indices!\n";
    std::vector<GraspSet> hypothesis_set_list;
    hypothesis_set_list.resize(0);
    return hypothesis_set_list;
  }

  if (plots_local_axes_)
    plot_.plotLocalAxes(frames, cloud_cam.getCloudOriginal());

  // 2. Evaluate possible hand placements.
  std::cout << "Finding hand poses ...\n";
  //针对找到的系列局部坐标系，来估计一些可能的抓取
  std::vector<GraspSet> hypothesis_set_list = evaluateHands(cloud_cam, frames, kdtree);

  std::cout << "====> HAND SEARCH TIME: " << omp_get_wtime() - t0_total << std::endl;

  return hypothesis_set_list;
}


std::vector<Grasp> HandSearch::reevaluateHypotheses(const CloudCamera& cloud_cam, const std::vector<Grasp>& grasps,
  bool plot_samples) const
{
  // create KdTree for neighborhood search
  const Eigen::MatrixXi& camera_source = cloud_cam.getCameraSource();
  const Eigen::Matrix3Xd& cloud_normals = cloud_cam.getNormals();
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  kdtree.setInputCloud(cloud);

  if (plot_samples)
  {
    Plot plotter;
    Eigen::Matrix3Xd samples(3, grasps.size());
    for (int i = 0; i < grasps.size(); i++)
    {
      samples.col(i) = grasps[i].getSample();
    }

    plotter.plotSamples(samples, cloud);
  }

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  PointList point_list(points, cloud_normals, camera_source, cloud_cam.getViewPoints());
  PointList nn_points;
  std::vector<int> labels(grasps.size()); // -1: not feasible, 0: feasible, >0: see Antipodal class

  #ifdef _OPENMP // parallelization using OpenMP
  #pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(params_.num_threads_)
  #endif
  for (int i = 0; i < grasps.size(); i++)
  {
    labels[i] = 0;
    pcl::PointXYZRGBA sample = eigenVectorToPcl(grasps[i].getSample());

    if (kdtree.radiusSearch(sample, nn_radius_, nn_indices, nn_dists) > 0)
    {
      nn_points = point_list.slice(nn_indices);
      nn_points.setPoints(nn_points.getPoints() - grasps[i].getSample().replicate(1, nn_points.size()));
      PointList nn_points_frame;
      FingerHand finger_hand(params_.finger_width_, params_.hand_outer_diameter_, params_.hand_depth_,params_.hand_height_);

      // set the lateral and forward axes of the robot hand frame (closing direction and grasp approach direction)
      if (params_.rotation_axis_ == ROTATION_AXIS_CURVATURE_AXIS)
      {
        finger_hand.setLateralAxis(1);
        finger_hand.setForwardAxis(0);
      }

      // check if the grasp is feasible (collision-free and contains at least one point)
      if (reevaluateHypothesis(nn_points, grasps[i], finger_hand, nn_points_frame))
      {
        // label the grasp
        labels[i] = labelHypothesis(nn_points_frame, finger_hand);
      }
    }
  }

  // remove empty list elements
  std::vector<Grasp> grasps_out;
  for (std::size_t i = 0; i < labels.size(); i++)
  {
    grasps_out.push_back(grasps[i]);
    bool is_half_grasp = labels[i] == Antipodal::FULL_GRASP || labels[i] == Antipodal::HALF_GRASP;
    grasps_out[grasps_out.size()-1].setHalfAntipodal(is_half_grasp);
    grasps_out[grasps_out.size()-1].setFullAntipodal(labels[i] == Antipodal::FULL_GRASP);
  }

  return grasps_out;
}


pcl::PointXYZRGBA HandSearch::eigenVectorToPcl(const Eigen::Vector3d& v) const
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}


std::vector<GraspSet> HandSearch::evaluateHands(const CloudCamera& cloud_cam, const std::vector<LocalFrame>& frames,
  const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const
{
  double t1 = omp_get_wtime();

  // possible angles used for hand orientations
  Eigen::VectorXd angles_space = Eigen::VectorXd::LinSpaced(params_.num_orientations_ + 1, -1.0 * M_PI/2.0, M_PI/2.0);

  // necessary b/c assignment in Eigen does not change vector size
  Eigen::VectorXd angles = angles_space.head(params_.num_orientations_);

  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  const PointCloudRGB::Ptr& cloud = cloud_cam.getCloudProcessed();
  Eigen::Matrix3Xd points = cloud->getMatrixXfMap().block(0, 0, 3, cloud->size()).cast<double>();
  std::vector<GraspSet> hand_set_list(frames.size());
  PointList point_list(points, cloud_cam.getNormals(), cloud_cam.getCameraSource(), cloud_cam.getViewPoints());
  PointList nn_points;
  Eigen::Matrix4d table_pose;
  if(cloud_cam.has_table)
  {
    table_pose = cloud_cam.getTablePose();
  }
  GraspSet::HandGeometry hand_geom(params_.finger_width_, params_.hand_outer_diameter_, params_.hand_depth_,
    params_.hand_height_, params_.init_bite_);
  //  GraspSet hand_set(hand_geom, angles, params_.rotation_axis_);

  #ifdef _OPENMP // parallelization using OpenMP
  #pragma omp parallel for private(nn_indices, nn_dists, nn_points) num_threads(params_.num_threads_)
  #endif
  //针对单个局部坐标系，计算候选抓取，并评估
  for (std::size_t i = 0; i < frames.size(); i++)
  {
    pcl::PointXYZRGBA sample = eigenVectorToPcl(frames[i].getSample());

    if (kdtree.radiusSearch(sample, nn_radius_, nn_indices, nn_dists) > 0)
    {
      nn_points = point_list.slice(nn_indices);
      //在这里对点云进行了平移，变成以局部坐标系为中心
      nn_points.setPoints(nn_points.getPoints() - frames[i].getSample().replicate(1, nn_points.size()));
      //对桌面坐标系也进行平移
      Eigen::Matrix4d table_pose_local;
      if(cloud_cam.has_table)
      {
        Eigen::MatrixXd temp(4,1);
        temp << frames[i].getSample()[0],frames[i].getSample()[1],frames[i].getSample()[2],1.0;
        table_pose_local = table_pose;
        table_pose_local.block(0,3,3,1) -= temp.block(0,0,3,1);
      }
      //根据旋转参数，初始化一些抓取配置
      GraspSet hand_set(hand_geom, angles, params_.rotation_axis_);
      //然后对这些局部坐标系进行评估，在hand_set内部就会针对frames[i]计算出一个候选抓取集合
      if(cloud_cam.has_table)
      {
        hand_set.evaluateHypotheses(nn_points, frames[i],table_pose_local);
      }
      else{
        hand_set.evaluateHypotheses(nn_points, frames[i]);
      }
      //


      if (hand_set.getIsValid().any()) // at least one feasible hand
      {
        hand_set_list[i] = hand_set;
      }
    }
  }

  // concatenate the grasp lists
  std::vector<GraspSet> hand_set_list_out;
  for (std::size_t i = 0; i < hand_set_list.size(); i++)
  {
    if (hand_set_list[i].getIsValid().any())
    {
      hand_set_list_out.push_back(hand_set_list[i]);
    }
  }

  double t2 = omp_get_wtime();
  std::cout << " Found " << hand_set_list_out.size() << " grasp candidate sets in " << t2 - t1 << " sec.\n";

  return hand_set_list_out;
}


bool HandSearch::reevaluateHypothesis(const PointList& point_list, const Grasp& hand,
  FingerHand& finger_hand, PointList& point_list_cropped) const
{
  // Transform points into hand frame and crop them on <hand_height>.
  PointList point_list_frame = point_list.rotatePointList(hand.getFrame().transpose());
  point_list_cropped = point_list_frame.cropByHandHeight(params_.hand_height_);

  // Check that the finger placement is possible.
  finger_hand.evaluateFingers(point_list_cropped.getPoints(), hand.getTop(), hand.getFingerPlacementIndex());
  finger_hand.evaluateHand(hand.getFingerPlacementIndex());

  if (finger_hand.getHand().any())
  {
    return true;
  }

  return false;
}


int HandSearch::labelHypothesis(const PointList& point_list, FingerHand& finger_hand) const
{
  std::vector<int> indices_learning = finger_hand.computePointsInClosingRegion(point_list.getPoints());
  if (indices_learning.size() == 0)
  {
    return Antipodal::NO_GRASP;
  }

  // extract data for classification
  PointList point_list_learning = point_list.slice(indices_learning);

  // evaluate if the grasp is antipodal
  Antipodal antipodal;
  int antipodal_result = antipodal.evaluateGrasp(point_list_learning, 0.003, finger_hand.getLateralAxis(),
    finger_hand.getForwardAxis(), params_.rotation_axis_);

  return antipodal_result;
}
