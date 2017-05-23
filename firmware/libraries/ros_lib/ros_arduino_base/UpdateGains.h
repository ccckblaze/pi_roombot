#ifndef _ROS_SERVICE_UpdateGains_h
#define _ROS_SERVICE_UpdateGains_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_arduino_base
{

static const char UPDATEGAINS[] = "ros_arduino_base/UpdateGains";

  class UpdateGainsRequest : public ros::Msg
  {
    public:
      float gains[3];

    UpdateGainsRequest():
      gains()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_gainsi;
      u_gainsi.real = this->gains[i];
      *(outbuffer + offset + 0) = (u_gainsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gainsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gainsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gainsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gains[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_gainsi;
      u_gainsi.base = 0;
      u_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gainsi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gains[i] = u_gainsi.real;
      offset += sizeof(this->gains[i]);
      }
     return offset;
    }

    const char * getType(){ return UPDATEGAINS; };
    const char * getMD5(){ return "8228f4ec4b23c46622122a1e302577ff"; };

  };

  class UpdateGainsResponse : public ros::Msg
  {
    public:

    UpdateGainsResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return UPDATEGAINS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class UpdateGains {
    public:
    typedef UpdateGainsRequest Request;
    typedef UpdateGainsResponse Response;
  };

}
#endif
