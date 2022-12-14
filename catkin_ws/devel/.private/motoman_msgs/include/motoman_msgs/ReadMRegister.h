// Generated by gencpp from file motoman_msgs/ReadMRegister.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_READMREGISTER_H
#define MOTOMAN_MSGS_MESSAGE_READMREGISTER_H

#include <ros/service_traits.h>


#include <motoman_msgs/ReadMRegisterRequest.h>
#include <motoman_msgs/ReadMRegisterResponse.h>


namespace motoman_msgs
{

struct ReadMRegister
{

typedef ReadMRegisterRequest Request;
typedef ReadMRegisterResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadMRegister
} // namespace motoman_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motoman_msgs::ReadMRegister > {
  static const char* value()
  {
    return "a3d35ac86126c0934861b8c43f69c8b8";
  }

  static const char* value(const ::motoman_msgs::ReadMRegister&) { return value(); }
};

template<>
struct DataType< ::motoman_msgs::ReadMRegister > {
  static const char* value()
  {
    return "motoman_msgs/ReadMRegister";
  }

  static const char* value(const ::motoman_msgs::ReadMRegister&) { return value(); }
};


// service_traits::MD5Sum< ::motoman_msgs::ReadMRegisterRequest> should match
// service_traits::MD5Sum< ::motoman_msgs::ReadMRegister >
template<>
struct MD5Sum< ::motoman_msgs::ReadMRegisterRequest>
{
  static const char* value()
  {
    return MD5Sum< ::motoman_msgs::ReadMRegister >::value();
  }
  static const char* value(const ::motoman_msgs::ReadMRegisterRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motoman_msgs::ReadMRegisterRequest> should match
// service_traits::DataType< ::motoman_msgs::ReadMRegister >
template<>
struct DataType< ::motoman_msgs::ReadMRegisterRequest>
{
  static const char* value()
  {
    return DataType< ::motoman_msgs::ReadMRegister >::value();
  }
  static const char* value(const ::motoman_msgs::ReadMRegisterRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motoman_msgs::ReadMRegisterResponse> should match
// service_traits::MD5Sum< ::motoman_msgs::ReadMRegister >
template<>
struct MD5Sum< ::motoman_msgs::ReadMRegisterResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motoman_msgs::ReadMRegister >::value();
  }
  static const char* value(const ::motoman_msgs::ReadMRegisterResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motoman_msgs::ReadMRegisterResponse> should match
// service_traits::DataType< ::motoman_msgs::ReadMRegister >
template<>
struct DataType< ::motoman_msgs::ReadMRegisterResponse>
{
  static const char* value()
  {
    return DataType< ::motoman_msgs::ReadMRegister >::value();
  }
  static const char* value(const ::motoman_msgs::ReadMRegisterResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_READMREGISTER_H
