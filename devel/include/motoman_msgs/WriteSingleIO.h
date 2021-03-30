// Generated by gencpp from file motoman_msgs/WriteSingleIO.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_WRITESINGLEIO_H
#define MOTOMAN_MSGS_MESSAGE_WRITESINGLEIO_H

#include <ros/service_traits.h>


#include <motoman_msgs/WriteSingleIORequest.h>
#include <motoman_msgs/WriteSingleIOResponse.h>


namespace motoman_msgs
{

struct WriteSingleIO
{

typedef WriteSingleIORequest Request;
typedef WriteSingleIOResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct WriteSingleIO
} // namespace motoman_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::motoman_msgs::WriteSingleIO > {
  static const char* value()
  {
    return "615ffdbac96f4fdce6e76dce79d5f3b5";
  }

  static const char* value(const ::motoman_msgs::WriteSingleIO&) { return value(); }
};

template<>
struct DataType< ::motoman_msgs::WriteSingleIO > {
  static const char* value()
  {
    return "motoman_msgs/WriteSingleIO";
  }

  static const char* value(const ::motoman_msgs::WriteSingleIO&) { return value(); }
};


// service_traits::MD5Sum< ::motoman_msgs::WriteSingleIORequest> should match 
// service_traits::MD5Sum< ::motoman_msgs::WriteSingleIO > 
template<>
struct MD5Sum< ::motoman_msgs::WriteSingleIORequest>
{
  static const char* value()
  {
    return MD5Sum< ::motoman_msgs::WriteSingleIO >::value();
  }
  static const char* value(const ::motoman_msgs::WriteSingleIORequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::motoman_msgs::WriteSingleIORequest> should match 
// service_traits::DataType< ::motoman_msgs::WriteSingleIO > 
template<>
struct DataType< ::motoman_msgs::WriteSingleIORequest>
{
  static const char* value()
  {
    return DataType< ::motoman_msgs::WriteSingleIO >::value();
  }
  static const char* value(const ::motoman_msgs::WriteSingleIORequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::motoman_msgs::WriteSingleIOResponse> should match 
// service_traits::MD5Sum< ::motoman_msgs::WriteSingleIO > 
template<>
struct MD5Sum< ::motoman_msgs::WriteSingleIOResponse>
{
  static const char* value()
  {
    return MD5Sum< ::motoman_msgs::WriteSingleIO >::value();
  }
  static const char* value(const ::motoman_msgs::WriteSingleIOResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::motoman_msgs::WriteSingleIOResponse> should match 
// service_traits::DataType< ::motoman_msgs::WriteSingleIO > 
template<>
struct DataType< ::motoman_msgs::WriteSingleIOResponse>
{
  static const char* value()
  {
    return DataType< ::motoman_msgs::WriteSingleIO >::value();
  }
  static const char* value(const ::motoman_msgs::WriteSingleIOResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_WRITESINGLEIO_H
