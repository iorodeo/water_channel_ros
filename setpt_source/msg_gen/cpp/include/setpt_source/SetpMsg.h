/* Auto-generated by genmsg_cpp for file /home/peter/mercurial/water_channel_ros/setpt_source/msg/SetpMsg.msg */
#ifndef SETPT_SOURCE_MESSAGE_SETPMSG_H
#define SETPT_SOURCE_MESSAGE_SETPMSG_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"

namespace setpt_source
{
template <class ContainerAllocator>
struct SetpMsg_ : public ros::Message
{
  typedef SetpMsg_<ContainerAllocator> Type;

  SetpMsg_()
  : header()
  , position(0.0)
  , velocity(0.0)
  , error(0.0)
  {
  }

  SetpMsg_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , position(0.0)
  , velocity(0.0)
  , error(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _position_type;
  float position;

  typedef float _velocity_type;
  float velocity;

  typedef float _error_type;
  float error;


private:
  static const char* __s_getDataType_() { return "setpt_source/SetpMsg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "10303c7fe9284b8def11c496985b6409"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
float32 position\n\
float32 velocity\n\
float32 error\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, position);
    ros::serialization::serialize(stream, velocity);
    ros::serialization::serialize(stream, error);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, position);
    ros::serialization::deserialize(stream, velocity);
    ros::serialization::deserialize(stream, error);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(position);
    size += ros::serialization::serializationLength(velocity);
    size += ros::serialization::serializationLength(error);
    return size;
  }

  typedef boost::shared_ptr< ::setpt_source::SetpMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::setpt_source::SetpMsg_<ContainerAllocator>  const> ConstPtr;
}; // struct SetpMsg
typedef  ::setpt_source::SetpMsg_<std::allocator<void> > SetpMsg;

typedef boost::shared_ptr< ::setpt_source::SetpMsg> SetpMsgPtr;
typedef boost::shared_ptr< ::setpt_source::SetpMsg const> SetpMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::setpt_source::SetpMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::setpt_source::SetpMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace setpt_source

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::setpt_source::SetpMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "10303c7fe9284b8def11c496985b6409";
  }

  static const char* value(const  ::setpt_source::SetpMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x10303c7fe9284b8dULL;
  static const uint64_t static_value2 = 0xef11c496985b6409ULL;
};

template<class ContainerAllocator>
struct DataType< ::setpt_source::SetpMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "setpt_source/SetpMsg";
  }

  static const char* value(const  ::setpt_source::SetpMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::setpt_source::SetpMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 position\n\
float32 velocity\n\
float32 error\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::setpt_source::SetpMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::setpt_source::SetpMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::setpt_source::SetpMsg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::setpt_source::SetpMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.position);
    stream.next(m.velocity);
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetpMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::setpt_source::SetpMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::setpt_source::SetpMsg_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    Printer<float>::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
    s << indent << "error: ";
    Printer<float>::stream(s, indent + "  ", v.error);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SETPT_SOURCE_MESSAGE_SETPMSG_H

