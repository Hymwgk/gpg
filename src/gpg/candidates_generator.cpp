#include <gpg/candidates_generator.h>


CandidatesGenerator::CandidatesGenerator(const Parameters& params,
  const HandSearch::Parameters& hand_search_params) : params_(params)
{
  Eigen::initParallel();

  hand_search_ = new HandSearch(hand_search_params);
}


void CandidatesGenerator::preprocessPointCloud(CloudCamera& cloud_cam)
{
//  const double VOXEL_SIZE = 0.002;均匀采样
  const double VOXEL_SIZE = 0.003;

  std::cout << "Processing cloud with: " << cloud_cam.getCloudOriginal()->size() << " points.\n";

  // Calculate surface normals using integral images if possible.
  /*
  if (cloud_cam.getCloudOriginal()->isOrganized() && cloud_cam.getNormals().cols() == 0)
  {
    cloud_cam.calculateNormals(0);
  }
  */

  // perform statistical outlier removal
  if (params_.remove_statistical_outliers_)
  {
    //    Plot plotter;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "before");

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud_cam.getCloudProcessed());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_cam.getCloudProcessed());
    std::cout << "Cloud after removing statistical outliers: " << cloud_cam.getCloudProcessed()->size() << std::endl;
    //    plotter.drawCloud(cloud_cam.getCloudProcessed(), "after");
  }


  // No indices into point cloud given
  if (cloud_cam.getSampleIndices().size() == 0)
  {
    // 1. Workspace filtering
    cloud_cam.filterWorkspace(params_.workspace_);
    std::cout << "After workspace filtering: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";

    if (cloud_cam.getCloudProcessed()->size() == 0)
    {
      return;
    }
    // 剔除桌面点云
    if (params_.remove_table_)
    {
      cloud_cam.sampleAbovePlane();
      std::cout << "After table filtering: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";
    }
    if (cloud_cam.getCloudProcessed()->size() == 0)
    {
      return;
    }


    // 2. Voxelization
    if (params_.voxelize_)
    {
      cloud_cam.voxelizeCloud(VOXEL_SIZE);
      std::cout << "After voxelization: " << cloud_cam.getCloudProcessed()->size() << " points left.\n";
    }

    // 3. Subsampling
    if (cloud_cam.getSamples().cols() > 0)
    {
      // 4. Calculate surface normals.计算表面法向量
      if (cloud_cam.getNormals().cols() == 0)
      {
        cloud_cam.calculateNormals(params_.num_threads_);
      }

      // 5. Subsample the remaining samples.
      cloud_cam.subsampleSamples(params_.num_samples_);
    }
    else
    {
      if (params_.num_samples_ > cloud_cam.getCloudProcessed()->size())
      {
        std::vector<int> indices_all(cloud_cam.getCloudProcessed()->size());
        for (int i=0; i < cloud_cam.getCloudProcessed()->size(); i++)
          indices_all[i] = i;
        cloud_cam.setSampleIndices(indices_all);
        std::cout << "Cloud is smaller than num_samples. Subsampled all " << cloud_cam.getCloudProcessed()->size()
          << " points.\n";
      }
      else
      {
        cloud_cam.subsampleUniformly(params_.num_samples_);
        std::cout << "Subsampled " << params_.num_samples_ << " at random uniformly.\n";
      }
    }
  }
  // Indices into point cloud given
  else
  {
    if (params_.num_samples_ > 0 && params_.num_samples_ < cloud_cam.getSampleIndices().size())
    {
      std::vector<int> indices_rand(params_.num_samples_);
      for (int i=0; i < params_.num_samples_; i++)
        indices_rand[i] = cloud_cam.getSampleIndices()[rand() % cloud_cam.getSampleIndices().size()];
      cloud_cam.setSampleIndices(indices_rand);
      std::cout << "Subsampled " << indices_rand.size() << " indices.\n";
    }
    else
    {
      std::cout << "Using all " << cloud_cam.getSampleIndices().size() << " indices.\n";
    }
  }

  // 4. Calculate surface normals.
  if (cloud_cam.getNormals().cols() == 0)
  {
    cloud_cam.calculateNormals(params_.num_threads_);
  }

  if (params_.plot_normals_)
  {
	  plotter_.plotNormals(cloud_cam.getCloudProcessed(), cloud_cam.getNormals());
  }
}


std::vector<Grasp> CandidatesGenerator::generateGraspCandidates(const CloudCamera& cloud_cam)
{
  // Find sets of grasp candidates.
  //如果设配置参数num_samples=n，那么计算查找出m<=n个抓取集合对象
  //每个抓取局部坐标系检测出0或1个集合
  std::vector<GraspSet> hand_set_list = hand_search_->searchHands(cloud_cam);
  std::cout << "Generated " << hand_set_list.size() << " grasp candidate sets.\n";

  // Extract the grasp candidates.把这些抓取从抓取集合类对象中抽取出来，组成一个大vector
  std::vector<Grasp> candidates;
  for (int i = 0; i < hand_set_list.size(); i++)
  {
    //得到第i个抓取集合里面的 候选抓取
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {
        candidates.push_back(hands[j]);
      }
    }
  }
  std::cout << "Generated " << candidates.size() << " grasp candidates.\n";

  if (params_.plot_grasps_)
  {
    const HandSearch::Parameters& params = hand_search_->getParams();
    plotter_.plotFingers3D(candidates, cloud_cam.getCloudProcessed(), "Grasp Candidates", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return candidates;
}

CandidatesGenerator::GraspsWithPoints CandidatesGenerator::generateGraspCandidatesWithInnerPoints(const CloudCamera& cloud_cam)
{
  // Find sets of grasp candidates.
  //如果设配置参数num_samples=n，那么计算查找出m<=n个抓取集合对象
  //每个抓取局部坐标系检测出0或1个集合
  std::vector<GraspSet> hand_set_list = hand_search_->searchHands(cloud_cam);
  std::cout << "Generated " << hand_set_list.size() << " grasp candidate sets.\n";

  // Extract the grasp candidates.把这些抓取从抓取集合类对象中抽取出来，组成一个大vector
  GraspsWithPoints grasps_with_points;
  //std::vector<Grasp> candidates;
  //std::vector<Eigen::Matrix3Xd> inner_points;
  for (int i = 0; i < hand_set_list.size(); i++)
  {
    //得到第i个抓取集合里面的 候选抓取
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();
    //得到第i个抓取集合里面的 候选抓取 对应的夹爪内部点云
    const std::vector<PointList>& hand_inner_points = hand_set_list[i].getInnerPoints();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {
        grasps_with_points.candidates.push_back(hands[j]);
        grasps_with_points.inner_points.push_back(hand_inner_points[j].getPoints());
      }
    }
  }
  std::cout << "Generated " << grasps_with_points.candidates.size() << " grasp candidates.\n";

  if (params_.plot_grasps_)
  {
    const HandSearch::Parameters& params = hand_search_->getParams();
    //plotter_.plotFingers3D(grasps_with_points.candidates, cloud_cam.getCloudProcessed(), "Grasp Candidates", params.hand_outer_diameter_,
    //  params.finger_width_, params.hand_depth_, params.hand_height_);
    plotter_.plotFingers3D(cloud_cam.getTablePose(),grasps_with_points.candidates, cloud_cam.getCloudProcessed(), "Grasp Candidates", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return grasps_with_points;
}


std::vector<GraspSet> CandidatesGenerator::generateGraspCandidateSets(const CloudCamera& cloud_cam)
{
  // Find sets of grasp candidates.
  std::vector<GraspSet> hand_set_list = hand_search_->searchHands(cloud_cam);

  if (params_.plot_grasps_)
  {
    const HandSearch::Parameters& params = hand_search_->getParams();
    plotter_.plotFingers3D(hand_set_list, cloud_cam.getCloudOriginal(), "Grasp Candidates", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return hand_set_list;
}


std::vector<Grasp> CandidatesGenerator::reevaluateHypotheses(const CloudCamera& cloud_cam,
  const std::vector<Grasp>& grasps)
{
  return hand_search_->reevaluateHypotheses(cloud_cam, grasps);
}
