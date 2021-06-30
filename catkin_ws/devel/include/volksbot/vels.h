// Generated by gencpp from file volksbot/vels.msg
// DO NOT EDIT!


#ifndef VOLKSBOT_MESSAGE_VELS_H
#define VOLKSBOT_MESSAGE_VELS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace volksbot
{
template <class ContainerAllocator>
struct vels_
{
  typedef vels_<ContainerAllocator> Type;

  vels_()
    : left(0.0)
    , right(0.0)
    , id(0)  {
    }
  vels_(const ContainerAllocator& _alloc)
    : left(0.0)
    , right(0.0)
    , id(0)  {
  (void)_alloc;
    }



   typedef double _left_type;
  _left_type left;

   typedef double _right_type;
  _right_type right;

   typedef int64_t _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::volksbot::vels_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::volksbot::vels_<ContainerAllocator> const> ConstPtr;

}; // struct vels_

typedef ::volksbot::vels_<std::allocator<void> > vels;

typedef boost::shared_ptr< ::volksbot::vels > velsPtr;
typedef boost::shared_ptr< ::volksbot::vels const> velsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::volksbot::vels_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::volksbot::vels_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::volksbot::vels_<ContainerAllocator1> & lhs, const ::volksbot::vels_<ContainerAllocator2> & rhs)
{
  return lhs.left == rhs.left &&
    lhs.right == rhs.right &&
    lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::volksbot::vels_<ContainerAllocator1> & lhs, const ::volksbot::vels_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace volksbot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::volksbot::vels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::volksbot::vels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::volksbot::vels_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::volksbot::vels_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::volksbot::vels_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::volksbot::vels_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::volksbot::vels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f87e64cfd56ef780d9b5e725006530e1";
  }

  static const char* value(const ::volksbot::vels_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf87e64cfd56ef780ULL;
  static const uint64_t static_value2 = 0xd9b5e725006530e1ULL;
};

template<class ContainerAllocator>
struct DataType< ::volksbot::vels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "volksbot/vels";
  }

  static const char* value(const ::volksbot::vels_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::volksbot::vels_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 left\n"
"float64 right\n"
"int64 id\n"
;
  }

  static const char* value(const ::volksbot::vels_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::volksbot::vels_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left);
      stream.next(m.right);
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct vels_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::volksbot::vels_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::volksbot::vels_<ContainerAllocator>& v)
  {
    s << indent << "left: ";
    Printer<double>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<double>::stream(s, indent + "  ", v.right);
    s << indent << "id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VOLKSBOT_MESSAGE_VELS_H
