// Generated by gencpp from file yolact_ros_msgs/Mask.msg
// DO NOT EDIT!


#ifndef YOLACT_ROS_MSGS_MESSAGE_MASK_H
#define YOLACT_ROS_MSGS_MESSAGE_MASK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace yolact_ros_msgs
{
template <class ContainerAllocator>
struct Mask_
{
  typedef Mask_<ContainerAllocator> Type;

  Mask_()
    : width(0)
    , height(0)
    , mask()  {
    }
  Mask_(const ContainerAllocator& _alloc)
    : width(0)
    , height(0)
    , mask(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _width_type;
  _width_type width;

   typedef int32_t _height_type;
  _height_type height;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _mask_type;
  _mask_type mask;





  typedef boost::shared_ptr< ::yolact_ros_msgs::Mask_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yolact_ros_msgs::Mask_<ContainerAllocator> const> ConstPtr;

}; // struct Mask_

typedef ::yolact_ros_msgs::Mask_<std::allocator<void> > Mask;

typedef boost::shared_ptr< ::yolact_ros_msgs::Mask > MaskPtr;
typedef boost::shared_ptr< ::yolact_ros_msgs::Mask const> MaskConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yolact_ros_msgs::Mask_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yolact_ros_msgs::Mask_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::yolact_ros_msgs::Mask_<ContainerAllocator1> & lhs, const ::yolact_ros_msgs::Mask_<ContainerAllocator2> & rhs)
{
  return lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.mask == rhs.mask;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::yolact_ros_msgs::Mask_<ContainerAllocator1> & lhs, const ::yolact_ros_msgs::Mask_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace yolact_ros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yolact_ros_msgs::Mask_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yolact_ros_msgs::Mask_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yolact_ros_msgs::Mask_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f8225f014f9a8dbca8b10e8e3685eb8c";
  }

  static const char* value(const ::yolact_ros_msgs::Mask_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf8225f014f9a8dbcULL;
  static const uint64_t static_value2 = 0xa8b10e8e3685eb8cULL;
};

template<class ContainerAllocator>
struct DataType< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yolact_ros_msgs/Mask";
  }

  static const char* value(const ::yolact_ros_msgs::Mask_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 width\n"
"int32 height\n"
"uint8[] mask # Mask encoded as bitset\n"
;
  }

  static const char* value(const ::yolact_ros_msgs::Mask_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.mask);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mask_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yolact_ros_msgs::Mask_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yolact_ros_msgs::Mask_<ContainerAllocator>& v)
  {
    s << indent << "width: ";
    Printer<int32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
    s << indent << "mask[]" << std::endl;
    for (size_t i = 0; i < v.mask.size(); ++i)
    {
      s << indent << "  mask[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.mask[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // YOLACT_ROS_MSGS_MESSAGE_MASK_H