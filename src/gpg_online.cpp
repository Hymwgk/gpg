// System
#include <sstream>
#include <string>
#include <vector>
//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom
#include <gpg/candidates_generator.h>
#include <gpg/hand_search.h>
#include <gpg/config_file.h>

using namespace std;
using namespace pcl;
// function to read in a double array from a single line of a configuration file
std::vector<double> stringToDouble(const std::string& str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  double v;

  while (ss >> v)
  {
    values.push_back(v);
    if (ss.peek() == ' ')
    {
      ss.ignore();
    }
  }

  return values;
}

class GPG_ONLINE
{
	public:
  	ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;

    tf::TransformListener listener;

    Eigen::Matrix4f transform_matrix;
    //生成器
    CandidatesGenerator *candidates_generator;

    Eigen::Matrix3Xd *view_points;

    Eigen::MatrixXi camera_source;

    //外部读取参数
    ConfigFile config_file;
    //生成器相关参数
    CandidatesGenerator::Parameters generator_params;
    //夹爪物理参数
    HandSearch::Parameters hand_search_params;
    std::vector<double> camera_pose;
    std::vector<double> table_pose;

  GPG_ONLINE(std::string config_path):config_file(config_path)
  {

    //读取夹爪物理参数
    double finger_width = config_file.getValueOfKey<double>("finger_width", 0.01);
    double hand_outer_diameter  = config_file.getValueOfKey<double>("hand_outer_diameter", 0.12);
    double hand_depth = config_file.getValueOfKey<double>("hand_depth", 0.06);
    double hand_height  = config_file.getValueOfKey<double>("hand_height", 0.02);
    double init_bite  = config_file.getValueOfKey<double>("init_bite", 0.01);

    std::cout << "finger_width: " << finger_width << "\n";
    std::cout << "hand_outer_diameter: " << hand_outer_diameter << "\n";
    std::cout << "hand_depth: " << hand_depth << "\n";
    std::cout << "hand_height: " << hand_height << "\n";
    std::cout << "init_bite: " << init_bite << "\n";
    //
    bool voxelize = config_file.getValueOfKey<bool>("voxelize", true);
    bool remove_outliers = config_file.getValueOfKey<bool>("remove_outliers", false);
    std::string workspace_str = config_file.getValueOfKeyAsString("workspace", "");
    std::string camera_pose_str = config_file.getValueOfKeyAsString("camera_pose", "");
    std::string table_pose_str = config_file.getValueOfKeyAsString("table_pose", "");
    std::vector<double> workspace = stringToDouble(workspace_str);
    camera_pose = stringToDouble(camera_pose_str);
    table_pose=stringToDouble(table_pose_str);
    std::cout << "voxelize: " << voxelize << "\n";
    std::cout << "remove_outliers: " << remove_outliers << "\n";
    std::cout << "workspace: " << workspace_str << "\n";
    std::cout << "camera_pose: " << camera_pose_str << "\n";

    int num_samples = config_file.getValueOfKey<int>("num_samples", 1000);
    int num_threads = config_file.getValueOfKey<int>("num_threads", 1);
    double nn_radius = config_file.getValueOfKey<double>("nn_radius", 0.01);
    int num_orientations = config_file.getValueOfKey<int>("num_orientations", 8);
    int rotation_axis = config_file.getValueOfKey<int>("rotation_axis", 2);
    std::cout << "num_samples: " << num_samples << "\n";
    std::cout << "num_threads: " << num_threads << "\n";
    std::cout << "nn_radius: " << nn_radius << "\n";
    std::cout << "num_orientations: " << num_orientations << "\n";
    std::cout << "rotation_axis: " << rotation_axis << "\n";

    bool plot_grasps = config_file.getValueOfKey<bool>("plot_grasps", true);
    bool plot_normals = config_file.getValueOfKey<bool>("plot_normals", false);
    std::cout << "plot_grasps: " << plot_grasps << "\n";
    std::cout << "plot_normals: " << plot_normals << "\n";

    //生成器相关参数
    generator_params.num_samples_ = num_samples;
    generator_params.num_threads_ = num_threads;
    generator_params.plot_normals_ = plot_normals;
    generator_params.plot_grasps_ = plot_grasps;
    generator_params.remove_statistical_outliers_ = remove_outliers;
    generator_params.voxelize_ = voxelize;
    generator_params.workspace_ = workspace;
    //夹爪对象参数
    hand_search_params.finger_width_ = finger_width;
    hand_search_params.hand_outer_diameter_ = hand_outer_diameter;
    hand_search_params.hand_depth_ = hand_depth;
    hand_search_params.hand_height_ = hand_height;
    hand_search_params.init_bite_ = init_bite;
    hand_search_params.nn_radius_frames_ = nn_radius;
    hand_search_params.num_orientations_ = num_orientations;
    hand_search_params.num_samples_ = num_samples;
    hand_search_params.num_threads_ = num_threads;
    hand_search_params.rotation_axis_ = rotation_axis;
    //初始化抓取生成器
    candidates_generator=new  CandidatesGenerator(generator_params, hand_search_params);
    //初始化并写入相机视点，默认(0,0,0)
    view_points = new Eigen::Matrix3Xd(3,1);
    //view_points->operator<<((camera_pose[3], camera_pose[6], camera_pose[9]));

    // Create object to load point cloud from file.
    //CloudCamera cloud_cam(argv[2], view_points);

    //订阅kinect点云话题
    sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1, &GPG_ONLINE::gpg,this);
  }

  void gpg (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    Eigen::Matrix3Xd view_points(3,1);
    view_points << camera_pose[3], camera_pose[6], camera_pose[9];
    //得到桌面标签坐标系
    Eigen::VectorXd table_pose_v;
    table_pose_v<<table_pose[0],table_pose[1],table_pose[2],table_pose[3],
      table_pose[4],table_pose[5],table_pose[6],table_pose[7],table_pose[8];
    Eigen::Matrix4d table_pose_m = Eigen::Map<Eigen::Matrix4d>(table_pose_v.data()).transpose();

    PointCloudRGB::Ptr cloud(new PointCloudRGB());
		//把kinect点云数据转化为pcl::PointXYZRGBA
		pcl::fromROSMsg	(*cloud_msg,*cloud);
    camera_source = Eigen::MatrixXi::Ones(1, cloud->size());
    //(点云+相机)整体对象
    CloudCamera cloud_cam(cloud,camera_source,view_points,table_pose_m);
    if (cloud_cam.getCloudOriginal()->size() == 0)
    {
      ROS_WARN("Input point cloud is empty or does not exist!\n");
    }
    // 点云预处理，包括点云体素降采样，移除离群点，工作空间滤除(直通滤波)，计算法向量Point cloud preprocessing: voxelize, remove statistical outliers, workspace filter, compute normals, subsample.
    candidates_generator->preprocessPointCloud(cloud_cam);

    //进行采样得到系列候选抓取 以及它们夹爪内部区域的点云集合
    CandidatesGenerator::GraspsWithPoints grasps_with_points= 
            candidates_generator->generateGraspCandidatesWithInnerPoints(cloud_cam);

  }


};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gpg_online");


  if (argc==2)  
  {
    GPG_ONLINE gpg_oneline(argv[1]);
    ros::spin();
  }
  else
  {
    ROS_INFO("参数过多，或未输入配置文件地址！\n");
  }
  return 0;
}
