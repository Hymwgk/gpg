// Generated by gencpp from file gpg/CloudIndexed.msg
// DO NOT EDIT!


#ifndef GPG_MESSAGE_CLOUDINDEXED_H
#define GPG_MESSAGE_CLOUDINDEXED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <gpg/CloudSources.h>
#include <std_msgs/Int64.h>

namespace gpg
{
template <class ContainerAllocator>
struct CloudIndexed_
{
  typedef CloudIndexed_<ContainerAllocator> Type;

  CloudIndexed_()
    : cloud_sources()
    , indices()  {
    }
  CloudIndexed_(const ContainerAllocator& _alloc)
    : cloud_sources(_alloc)
    , indices(_alloc)  {
  (void)_alloc;
    }



   typedef  ::gpg::CloudSources_<ContainerAllocator>  _cloud_sources_type;
  _cloud_sources_type cloud_sources;

   typedef std::vector< ::std_msgs::Int64_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::Int64_<ContainerAllocator> >::other >  _indices_type;
  _indices_type indices;





  typedef boost::shared_ptr< ::gpg::CloudIndexed_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gpg::CloudIndexed_<ContainerAllocator> const> ConstPtr;

}; // struct CloudIndexed_

typedef ::gpg::CloudIndexed_<std::allocator<void> > CloudIndexed;

typedef boost::shared_ptr< ::gpg::CloudIndexed > CloudIndexedPtr;
typedef boost::shared_ptr< ::gpg::CloudIndexed const> CloudIndexedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gpg::CloudIndexed_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gpg::CloudIndexed_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gpg::CloudIndexed_<ContainerAllocator1> & lhs, const ::gpg::CloudIndexed_<ContainerAllocator2> & rhs)
{
  return lhs.cloud_sources == rhs.cloud_sources &&
    lhs.indices == rhs.indices;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gpg::CloudIndexed_<ContainerAllocator1> & lhs, const ::gpg::CloudIndexed_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gpg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::gpg::CloudIndexed_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gpg::CloudIndexed_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gpg::CloudIndexed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gpg::CloudIndexed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gpg::CloudIndexed_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gpg::CloudIndexed_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gpg::CloudIndexed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e30f3eb59956952b459cd77778d0412d";
  }

  static const char* value(const ::gpg::CloudIndexed_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe30f3eb59956952bULL;
  static const uint64_t static_value2 = 0x459cd77778d0412dULL;
};

template<class ContainerAllocator>
struct DataType< ::gpg::CloudIndexed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gpg/CloudIndexed";
  }

  static const char* value(const ::gpg::CloudIndexed_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gpg::CloudIndexed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message holds a point cloud and a list of indices into the point cloud \n"
"# at which to sample grasp candidates.\n"
"\n"
"# The point cloud.\n"
"gpg/CloudSources cloud_sources\n"
"\n"
"# The indices into the point cloud at which to sample grasp candidates.\n"
"std_msgs/Int64[] indices\n"
"\n"
"================================================================================\n"
"MSG: gpg/CloudSources\n"
"# This message holds a point cloud that can be a combination of point clouds \n"
"# from different camera sources (at least one). For each point in the cloud, \n"
"# this message also stores the index of the camera that produced the point.\n"
"\n"
"# The point cloud.\n"
"sensor_msgs/PointCloud2 cloud\n"
"\n"
"# For each point in the cloud, the index of the camera that acquired the point.\n"
"std_msgs/Int64[] camera_source\n"
"\n"
"# A list of camera positions at which the point cloud was acquired.\n"
"geometry_msgs/Point[] view_points\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Int64\n"
"int64 data\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::gpg::CloudIndexed_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gpg::CloudIndexed_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cloud_sources);
      stream.next(m.indices);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CloudIndexed_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gpg::CloudIndexed_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gpg::CloudIndexed_<ContainerAllocator>& v)
  {
    s << indent << "cloud_sources: ";
    s << std::endl;
    Printer< ::gpg::CloudSources_<ContainerAllocator> >::stream(s, indent + "  ", v.cloud_sources);
    s << indent << "indices[]" << std::endl;
    for (size_t i = 0; i < v.indices.size(); ++i)
    {
      s << indent << "  indices[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::Int64_<ContainerAllocator> >::stream(s, indent + "    ", v.indices[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPG_MESSAGE_CLOUDINDEXED_H
