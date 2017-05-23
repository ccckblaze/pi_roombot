#ifndef _ROS_SERVICE_test_srv_h
#define _ROS_SERVICE_test_srv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pi_roombot
{

static const char TEST_SRV[] = "pi_roombot/test_srv";

  class test_srvRequest : public ros::Msg
  {
    public:
      int32_t A;
      int32_t B;
      int32_t C;

    test_srvRequest():
      A(0),
      B(0),
      C(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_A;
      u_A.real = this->A;
      *(outbuffer + offset + 0) = (u_A.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_A.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_A.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_A.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->A);
      union {
        int32_t real;
        uint32_t base;
      } u_B;
      u_B.real = this->B;
      *(outbuffer + offset + 0) = (u_B.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_B.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_B.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_B.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->B);
      union {
        int32_t real;
        uint32_t base;
      } u_C;
      u_C.real = this->C;
      *(outbuffer + offset + 0) = (u_C.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_C.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_C.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_C.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->C);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_A;
      u_A.base = 0;
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_A.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->A = u_A.real;
      offset += sizeof(this->A);
      union {
        int32_t real;
        uint32_t base;
      } u_B;
      u_B.base = 0;
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_B.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->B = u_B.real;
      offset += sizeof(this->B);
      union {
        int32_t real;
        uint32_t base;
      } u_C;
      u_C.base = 0;
      u_C.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_C.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_C.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_C.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->C = u_C.real;
      offset += sizeof(this->C);
     return offset;
    }

    const char * getType(){ return TEST_SRV; };
    const char * getMD5(){ return "e7a68ce4e0b75a9719b4950a7069c9d4"; };

  };

  class test_srvResponse : public ros::Msg
  {
    public:
      int32_t sum;

    test_srvResponse():
      sum(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sum;
      u_sum.real = this->sum;
      *(outbuffer + offset + 0) = (u_sum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_sum;
      u_sum.base = 0;
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sum = u_sum.real;
      offset += sizeof(this->sum);
     return offset;
    }

    const char * getType(){ return TEST_SRV; };
    const char * getMD5(){ return "0ba699c25c9418c0366f3595c0c8e8ec"; };

  };

  class test_srv {
    public:
    typedef test_srvRequest Request;
    typedef test_srvResponse Response;
  };

}
#endif
