#include <gpg/finger_hand.h>


FingerHand::FingerHand(double finger_width, double hand_outer_diameter,	double hand_depth,double hand_height)
  :	finger_width_(finger_width), hand_depth_(hand_depth),	hand_height_(hand_height),hand_outer_diameter_(hand_outer_diameter),lateral_axis_(-1),
   	forward_axis_(-1)
{
	int n = 10; // number of finger placements to consider over a single hand diameter

	// Calculate the finger spacing.
	Eigen::VectorXd fs_half;
  //生成一个长度为n的向量，元素值在0~(hand_outer_diameter - finger_width)之间均匀分布，首尾均包含
	fs_half.setLinSpaced(n, 0.0, hand_outer_diameter - finger_width);
	finger_spacing_.resize(2 * n);
  //内部值范围变成了2n份，从-(hand_outer_diameter - finger_width)~(hand_outer_diameter - finger_width)
	finger_spacing_	<< (fs_half.array() - hand_outer_diameter + finger_width_).matrix(), fs_half;

	fingers_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, 2 * n, false);
	hand_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, n, false);

  Eigen::Vector3d bottom_center(0.0,0.0,0.0);
  Eigen::Vector3d approach(1.0,0.0,0.0);
  Eigen::Vector3d binormal(0.0,1.0,0.0);
  //根据给定的夹爪参数，计算虚拟夹爪每个角点的局部坐标
  hand_points_ = FingerHand::getHandPoints(bottom_center,approach,binormal);

}


void FingerHand::evaluateFingers(const Eigen::Matrix3Xd& points, double bite, int idx)
{
  // Calculate top and bottom of the hand (top = fingertip, bottom = base).
  top_ = bite;
  bottom_ = bite - hand_depth_;

  fingers_.setConstant(false);

  // Crop points at bite.
  std::vector<int> cropped_indices;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(forward_axis_, i) < bite)
    {
      // Check that the hand would be able to extend by <bite> onto the object without causing the back of the hand to
      // collide with <points>.
      if (points(forward_axis_, i) < bottom_)
      {
        return;
      }

      cropped_indices.push_back(i);
    }
  }

  // Check that there is at least one point in between the fingers.
  if (cropped_indices.size() == 0)
  {
    return;
  }

  // Identify free gaps (finger placements that do not collide with the point cloud).
  if (idx == -1)
  {
    for (int i = 0; i < fingers_.size(); i++)
    {
      if (isGapFree(points, cropped_indices, i))
      {
        fingers_(i) = true;
      }
    }
  }
  else
  {
    if (isGapFree(points, cropped_indices, idx))
    {
      fingers_(idx) = true;
    }

    if (isGapFree(points, cropped_indices, fingers_.size() / 2 + idx))
    {
      fingers_(fingers_.size() / 2 + idx) = true;
    }
  }




}


void FingerHand::evaluateHand()
{
  const int n = fingers_.size() / 2;

  for (int i = 0; i < n; i++)
  {
    hand_(i) = (fingers_(i) == true && fingers_(n + i) == true);
  }
}

void FingerHand::collisionCheckHandTable(const Eigen::Matrix4d& table_pose_frame)
{
  const int n = hand_.size();

  for (int i = 0; i < n; i++)
  {
    //构建偏移向量
    Eigen::Vector3d offset;
    offset << 0.0,finger_spacing_[i],0.0;

    //对夹爪上的每个点坐标进行偏移
    Eigen::Matrix3Xd hand_points_collision = hand_points_;
    hand_points_collision.rowwise()+=offset.transpose();

    //检查偏移后的夹爪各点是否与桌面碰撞
    //计算桌面标签原点指向夹爪各点的向量
    hand_points_collision.rowwise() -=table_pose_frame.block(0,3,3,1).transpose();
    //计算各个向量与桌面标签z轴夹角
    //对每个行向量都单位化
    hand_points_collision.rowwise().normalize();
    //broadcast 以及 对应元素相乘
    hand_points_collision = hand_points_collision.cwiseProduct(table_pose_frame.block(0,3,3,1).transpose());
    //每行都求和
    Eigen::VectorXd row_sum= hand_points_collision.rowwise().sum();
    //找出最小值
    double min_r = row_sum.minCoeff();
    //如果最小值>=0就代表，所有角点都位于桌面之上;否则，与桌面有碰撞
    if(min_r<0)
    {
      hand_[i]=false;
    }
  }
}


void FingerHand::evaluateHand(int idx)
{
  const int n = fingers_.size() / 2;
  hand_.setConstant(false);
  hand_(idx) = (fingers_(idx) == true && fingers_(n + idx) == true);
}


int FingerHand::deepenHand(const Eigen::Matrix3Xd& points, double min_depth, double max_depth)
{
  std::vector<int> hand_idx;

  for (int i = 0; i < hand_.cols(); i++)
  {
    if (hand_(i) == true)
    {
      hand_idx.push_back(i);
    }
  }

  if (hand_idx.size() == 0)
  {
    return -1;
  }

  // Choose middle hand.
  int hand_eroded_idx = hand_idx[ceil(hand_idx.size() / 2.0) - 1]; // middle index
  int opposite_idx = fingers_.size() / 2 + hand_eroded_idx; // opposite finger index

  // Attempt to deepen hand (move as far onto the object as possible without collision).
  const double DEEPEN_STEP_SIZE = 0.005;
  FingerHand new_hand = *this;
  FingerHand last_new_hand = new_hand;

  for (double depth = min_depth + DEEPEN_STEP_SIZE; depth <= max_depth; depth += DEEPEN_STEP_SIZE)
  {
    // Check if the new hand placement is feasible
    new_hand.evaluateFingers(points, depth, hand_eroded_idx);
    if (!new_hand.fingers_(hand_eroded_idx) || !new_hand.fingers_(opposite_idx))
    {
      break;
    }

    hand_(hand_eroded_idx) = true;
    last_new_hand = new_hand;
  }

  // Recover the deepest hand.
  *this = last_new_hand;
  hand_.setConstant(false);
  hand_(hand_eroded_idx) = true;

  return hand_eroded_idx;
}


std::vector<int> FingerHand::computePointsInClosingRegion(const Eigen::Matrix3Xd& points, int idx)
{
  // Find feasible finger placement.
  if (idx == -1)
  {
    for (int i = 0; i < hand_.cols(); i++)
    {
      if (hand_(i) == true)
      {
        idx = i;
        break;
      }
    }
  }

  // Calculate the lateral parameters of the hand closing region for this finger placement.
  //对夹爪在binormal轴偏移到给定的位置
  left_ = finger_spacing_(idx) + finger_width_;
  right_ = finger_spacing_(hand_.cols() + idx);
  center_ = 0.5 * (left_ + right_);
  surface_ = points.row(lateral_axis_).minCoeff();

  // Find points inside the hand closing region defined by <bottom_>, <top_>, <left_> and <right_>.
  //
  std::vector<int> indices;
  for (int i = 0; i < points.cols(); i++)
  {
    //由于输入的点云已经是切片好的，因此，只需要判断
    //某点是否在binormal和approach轴的方向，处于夹爪内部即可
    if (points(forward_axis_, i) > bottom_ && points(forward_axis_, i) < top_
        && points(lateral_axis_, i) > left_ && points(lateral_axis_, i) < right_)
    {
      indices.push_back(i);
    }
  }

  return indices;
}

Eigen::Matrix3Xd FingerHand::getHandPoints(const Eigen::Vector3d &bottom_center,
  const Eigen::Vector3d &approach,const Eigen::Vector3d &binormal)
{
  double hh = hand_height_;
  double fw = finger_width_;
  double hod = hand_outer_diameter_;
  double hd = hand_depth_;
  //
  double open_w = hod -fw*2;
  //对接近轴和binormal轴进行单位化
  Eigen::Vector3d approach_normal=approach.normalized();
  Eigen::Vector3d binormal_normal = binormal.normalized();
  //
  Eigen::Vector3d minor_pc= approach_normal.cross(binormal_normal);
  //对向量进行单位化并返回一个新的向量
  //minor_pc = minor_pc.normalized();
  //单位化，并对自身进行修改
  minor_pc.normalize();

  Eigen::Vector3d p5_p6 = minor_pc * hh * 0.5 + bottom_center;
  Eigen::Vector3d p7_p8 = -minor_pc * hh * 0.5 + bottom_center;
  Eigen::Vector3d p5 = -binormal_normal * open_w * 0.5 + p5_p6;
  Eigen::Vector3d p6 = binormal_normal * open_w * 0.5 + p5_p6;
  Eigen::Vector3d p7 = binormal_normal * open_w * 0.5 + p7_p8;
  Eigen::Vector3d p8 = -binormal_normal * open_w * 0.5 + p7_p8;
  Eigen::Vector3d p1 = approach_normal * hd + p5;
  Eigen::Vector3d p2 = approach_normal * hd + p6;
  Eigen::Vector3d p3 = approach_normal * hd + p7;
  Eigen::Vector3d p4 = approach_normal * hd + p8;

  Eigen::Vector3d p9 = -binormal_normal * fw + p1;
  Eigen::Vector3d p10 = -binormal_normal * fw + p4;
  Eigen::Vector3d p11 = -binormal_normal * fw + p5;
  Eigen::Vector3d p12 = -binormal_normal * fw + p8;
  Eigen::Vector3d p13 = binormal_normal * fw + p2;
  Eigen::Vector3d p14 = binormal_normal * fw + p3;
  Eigen::Vector3d p15 = binormal_normal * fw + p6;
  Eigen::Vector3d p16 = binormal_normal * fw + p7;

  Eigen::Vector3d p17 = -approach_normal * hh + p11;
  Eigen::Vector3d p18 = -approach_normal * hh + p15;
  Eigen::Vector3d p19 = -approach_normal * hh + p16;
  Eigen::Vector3d p20 = -approach_normal * hh + p12;
  Eigen::Vector3d p0(0.0,0.0,0.0);

  Eigen::VectorXd p_v(p0.size()+p1.size()+p2.size()+p3.size()+p4.size()+p5.size()+p6.size()+p7.size()
    +p8.size()+p9.size()+p10.size()+p11.size()+p12.size()+p13.size()+p14.size()+p15.size()+p16.size()
    +p17.size()+p18.size()+p19.size()+p20.size());
  
  p_v << p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17,p18,p19,p20;
  //得到手指各点的矩阵(n*3)
  Eigen::Matrix3Xd p_m = Eigen::Map<Eigen::Matrix<double, 3,Dynamic,RowMajor>>(p_v.data());
  
  return p_m;
}

bool FingerHand::isGapFree(const Eigen::Matrix3Xd& points, const std::vector<int>& indices, int idx)
{
  for (int i = 0; i < indices.size(); i++)
  {
    const double& x = points(lateral_axis_, indices[i]);

    if (x > finger_spacing_(idx) && x < finger_spacing_(idx) + finger_width_)
    {
      return false;
    }
  }

  return true;
}
