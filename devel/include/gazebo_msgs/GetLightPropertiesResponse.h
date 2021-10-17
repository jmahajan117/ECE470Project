// Generated by gencpp from file gazebo_msgs/GetLightPropertiesResponse.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_GETLIGHTPROPERTIESRESPONSE_H
#define GAZEBO_MSGS_MESSAGE_GETLIGHTPROPERTIESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/ColorRGBA.h>

namespace gazebo_msgs
{
template <class ContainerAllocator>
struct GetLightPropertiesResponse_
{
  typedef GetLightPropertiesResponse_<ContainerAllocator> Type;

  GetLightPropertiesResponse_()
    : diffuse()
    , attenuation_constant(0.0)
    , attenuation_linear(0.0)
    , attenuation_quadratic(0.0)
    , success(false)
    , status_message()  {
    }
  GetLightPropertiesResponse_(const ContainerAllocator& _alloc)
    : diffuse(_alloc)
    , attenuation_constant(0.0)
    , attenuation_linear(0.0)
    , attenuation_quadratic(0.0)
    , success(false)
    , status_message(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _diffuse_type;
  _diffuse_type diffuse;

   typedef double _attenuation_constant_type;
  _attenuation_constant_type attenuation_constant;

   typedef double _attenuation_linear_type;
  _attenuation_linear_type attenuation_linear;

   typedef double _attenuation_quadratic_type;
  _attenuation_quadratic_type attenuation_quadratic;

   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_message_type;
  _status_message_type status_message;





  typedef boost::shared_ptr< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetLightPropertiesResponse_

typedef ::gazebo_msgs::GetLightPropertiesResponse_<std::allocator<void> > GetLightPropertiesResponse;

typedef boost::shared_ptr< ::gazebo_msgs::GetLightPropertiesResponse > GetLightPropertiesResponsePtr;
typedef boost::shared_ptr< ::gazebo_msgs::GetLightPropertiesResponse const> GetLightPropertiesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gazebo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'gazebo_msgs': ['/home/ur3/catkin_jaym2/src/drivers/gazebo_ros_pkgs/gazebo_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9a19ddd5aab4c13b7643d1722c709f1f";
  }

  static const char* value(const ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9a19ddd5aab4c13bULL;
  static const uint64_t static_value2 = 0x7643d1722c709f1fULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/GetLightPropertiesResponse";
  }

  static const char* value(const ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/ColorRGBA diffuse\n\
float64 attenuation_constant\n\
float64 attenuation_linear\n\
float64 attenuation_quadratic\n\
bool success\n\
string status_message\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/ColorRGBA\n\
float32 r\n\
float32 g\n\
float32 b\n\
float32 a\n\
";
  }

  static const char* value(const ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.diffuse);
      stream.next(m.attenuation_constant);
      stream.next(m.attenuation_linear);
      stream.next(m.attenuation_quadratic);
      stream.next(m.success);
      stream.next(m.status_message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetLightPropertiesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_msgs::GetLightPropertiesResponse_<ContainerAllocator>& v)
  {
    s << indent << "diffuse: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.diffuse);
    s << indent << "attenuation_constant: ";
    Printer<double>::stream(s, indent + "  ", v.attenuation_constant);
    s << indent << "attenuation_linear: ";
    Printer<double>::stream(s, indent + "  ", v.attenuation_linear);
    s << indent << "attenuation_quadratic: ";
    Printer<double>::stream(s, indent + "  ", v.attenuation_quadratic);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "status_message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status_message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_GETLIGHTPROPERTIESRESPONSE_H
