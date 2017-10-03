// Generated by gencpp from file whirlybird_msgs/Command.msg
// DO NOT EDIT!


#ifndef WHIRLYBIRD_MSGS_MESSAGE_COMMAND_H
#define WHIRLYBIRD_MSGS_MESSAGE_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace whirlybird_msgs
{
template <class ContainerAllocator>
struct Command_
{
  typedef Command_<ContainerAllocator> Type;

  Command_()
    : left_motor(0.0)
    , right_motor(0.0)  {
    }
  Command_(const ContainerAllocator& _alloc)
    : left_motor(0.0)
    , right_motor(0.0)  {
  (void)_alloc;
    }



   typedef float _left_motor_type;
  _left_motor_type left_motor;

   typedef float _right_motor_type;
  _right_motor_type right_motor;




  typedef boost::shared_ptr< ::whirlybird_msgs::Command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::whirlybird_msgs::Command_<ContainerAllocator> const> ConstPtr;

}; // struct Command_

typedef ::whirlybird_msgs::Command_<std::allocator<void> > Command;

typedef boost::shared_ptr< ::whirlybird_msgs::Command > CommandPtr;
typedef boost::shared_ptr< ::whirlybird_msgs::Command const> CommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::whirlybird_msgs::Command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::whirlybird_msgs::Command_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace whirlybird_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'whirlybird_msgs': ['/fsg/romer3/controls/whirlybird_ws/src/whirlybird_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::whirlybird_msgs::Command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::whirlybird_msgs::Command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whirlybird_msgs::Command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whirlybird_msgs::Command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whirlybird_msgs::Command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whirlybird_msgs::Command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::whirlybird_msgs::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e3717ac8e9443aa62d7102a5860f5e7";
  }

  static const char* value(const ::whirlybird_msgs::Command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e3717ac8e9443aaULL;
  static const uint64_t static_value2 = 0x62d7102a5860f5e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::whirlybird_msgs::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "whirlybird_msgs/Command";
  }

  static const char* value(const ::whirlybird_msgs::Command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::whirlybird_msgs::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Whirlybird.msg\n\
\n\
float32 left_motor\n\
float32 right_motor\n\
\n\
";
  }

  static const char* value(const ::whirlybird_msgs::Command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::whirlybird_msgs::Command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left_motor);
      stream.next(m.right_motor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::whirlybird_msgs::Command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::whirlybird_msgs::Command_<ContainerAllocator>& v)
  {
    s << indent << "left_motor: ";
    Printer<float>::stream(s, indent + "  ", v.left_motor);
    s << indent << "right_motor: ";
    Printer<float>::stream(s, indent + "  ", v.right_motor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WHIRLYBIRD_MSGS_MESSAGE_COMMAND_H