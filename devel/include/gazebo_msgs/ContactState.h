// Generated by gencpp from file gazebo_msgs/ContactState.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_CONTACTSTATE_H
#define GAZEBO_MSGS_MESSAGE_CONTACTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo_msgs
{
template <class ContainerAllocator>
struct ContactState_
{
  typedef ContactState_<ContainerAllocator> Type;

  ContactState_()
    : info()
    , collision1_name()
    , collision2_name()
    , wrenches()
    , total_wrench()
    , contact_positions()
    , contact_normals()
    , depths()  {
    }
  ContactState_(const ContainerAllocator& _alloc)
    : info(_alloc)
    , collision1_name(_alloc)
    , collision2_name(_alloc)
    , wrenches(_alloc)
    , total_wrench(_alloc)
    , contact_positions(_alloc)
    , contact_normals(_alloc)
    , depths(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _info_type;
  _info_type info;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _collision1_name_type;
  _collision1_name_type collision1_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _collision2_name_type;
  _collision2_name_type collision2_name;

   typedef std::vector< ::geometry_msgs::Wrench_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Wrench_<ContainerAllocator> >::other >  _wrenches_type;
  _wrenches_type wrenches;

   typedef  ::geometry_msgs::Wrench_<ContainerAllocator>  _total_wrench_type;
  _total_wrench_type total_wrench;

   typedef std::vector< ::geometry_msgs::Vector3_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Vector3_<ContainerAllocator> >::other >  _contact_positions_type;
  _contact_positions_type contact_positions;

   typedef std::vector< ::geometry_msgs::Vector3_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Vector3_<ContainerAllocator> >::other >  _contact_normals_type;
  _contact_normals_type contact_normals;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _depths_type;
  _depths_type depths;




  typedef boost::shared_ptr< ::gazebo_msgs::ContactState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_msgs::ContactState_<ContainerAllocator> const> ConstPtr;

}; // struct ContactState_

typedef ::gazebo_msgs::ContactState_<std::allocator<void> > ContactState;

typedef boost::shared_ptr< ::gazebo_msgs::ContactState > ContactStatePtr;
typedef boost::shared_ptr< ::gazebo_msgs::ContactState const> ContactStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_msgs::ContactState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_msgs::ContactState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace gazebo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/indigo/share/trajectory_msgs/cmake/../msg'], 'gazebo_msgs': ['/home/sathya/dynamics_ws/src/gazebo_ros_pkgs/gazebo_msgs/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::ContactState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_msgs::ContactState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::ContactState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_msgs::ContactState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::ContactState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_msgs::ContactState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "48c0ffb054b8c444f870cecea1ee50d9";
  }

  static const char* value(const ::gazebo_msgs::ContactState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x48c0ffb054b8c444ULL;
  static const uint64_t static_value2 = 0xf870cecea1ee50d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_msgs/ContactState";
  }

  static const char* value(const ::gazebo_msgs::ContactState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_msgs::ContactState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string info                                   # text info on this contact\n\
string collision1_name                        # name of contact collision1\n\
string collision2_name                        # name of contact collision2\n\
geometry_msgs/Wrench[] wrenches               # list of forces/torques\n\
geometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF\n\
geometry_msgs/Vector3[] contact_positions     # list of contact position\n\
geometry_msgs/Vector3[] contact_normals       # list of contact normals\n\
float64[] depths                              # list of penetration depths\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Wrench\n\
# This represents force in free space, separated into\n\
# its linear and angular parts.\n\
Vector3  force\n\
Vector3  torque\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::gazebo_msgs::ContactState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_msgs::ContactState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.info);
      stream.next(m.collision1_name);
      stream.next(m.collision2_name);
      stream.next(m.wrenches);
      stream.next(m.total_wrench);
      stream.next(m.contact_positions);
      stream.next(m.contact_normals);
      stream.next(m.depths);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ContactState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_msgs::ContactState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_msgs::ContactState_<ContainerAllocator>& v)
  {
    s << indent << "info: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.info);
    s << indent << "collision1_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.collision1_name);
    s << indent << "collision2_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.collision2_name);
    s << indent << "wrenches[]" << std::endl;
    for (size_t i = 0; i < v.wrenches.size(); ++i)
    {
      s << indent << "  wrenches[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >::stream(s, indent + "    ", v.wrenches[i]);
    }
    s << indent << "total_wrench: ";
    s << std::endl;
    Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >::stream(s, indent + "  ", v.total_wrench);
    s << indent << "contact_positions[]" << std::endl;
    for (size_t i = 0; i < v.contact_positions.size(); ++i)
    {
      s << indent << "  contact_positions[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "    ", v.contact_positions[i]);
    }
    s << indent << "contact_normals[]" << std::endl;
    for (size_t i = 0; i < v.contact_normals.size(); ++i)
    {
      s << indent << "  contact_normals[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "    ", v.contact_normals[i]);
    }
    s << indent << "depths[]" << std::endl;
    for (size_t i = 0; i < v.depths.size(); ++i)
    {
      s << indent << "  depths[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.depths[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_CONTACTSTATE_H
