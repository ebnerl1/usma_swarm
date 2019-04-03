/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/odroid/acs_ros_ws/src/autonomy-payload/ap_srvs/srv/SetInteger.srv
 *
 */


#ifndef AP_SRVS_MESSAGE_SETINTEGER_H
#define AP_SRVS_MESSAGE_SETINTEGER_H

#include <ros/service_traits.h>


#include <ap_srvs/SetIntegerRequest.h>
#include <ap_srvs/SetIntegerResponse.h>


namespace ap_srvs
{

struct SetInteger
{

typedef SetIntegerRequest Request;
typedef SetIntegerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetInteger
} // namespace ap_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ap_srvs::SetInteger > {
  static const char* value()
  {
    return "9cc7b8927f792e461dca9cd6251d1d23";
  }

  static const char* value(const ::ap_srvs::SetInteger&) { return value(); }
};

template<>
struct DataType< ::ap_srvs::SetInteger > {
  static const char* value()
  {
    return "ap_srvs/SetInteger";
  }

  static const char* value(const ::ap_srvs::SetInteger&) { return value(); }
};


// service_traits::MD5Sum< ::ap_srvs::SetIntegerRequest> should match 
// service_traits::MD5Sum< ::ap_srvs::SetInteger > 
template<>
struct MD5Sum< ::ap_srvs::SetIntegerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ap_srvs::SetInteger >::value();
  }
  static const char* value(const ::ap_srvs::SetIntegerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ap_srvs::SetIntegerRequest> should match 
// service_traits::DataType< ::ap_srvs::SetInteger > 
template<>
struct DataType< ::ap_srvs::SetIntegerRequest>
{
  static const char* value()
  {
    return DataType< ::ap_srvs::SetInteger >::value();
  }
  static const char* value(const ::ap_srvs::SetIntegerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ap_srvs::SetIntegerResponse> should match 
// service_traits::MD5Sum< ::ap_srvs::SetInteger > 
template<>
struct MD5Sum< ::ap_srvs::SetIntegerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ap_srvs::SetInteger >::value();
  }
  static const char* value(const ::ap_srvs::SetIntegerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ap_srvs::SetIntegerResponse> should match 
// service_traits::DataType< ::ap_srvs::SetInteger > 
template<>
struct DataType< ::ap_srvs::SetIntegerResponse>
{
  static const char* value()
  {
    return DataType< ::ap_srvs::SetInteger >::value();
  }
  static const char* value(const ::ap_srvs::SetIntegerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // AP_SRVS_MESSAGE_SETINTEGER_H