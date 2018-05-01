#ifndef _ROS_command2ros_Digger_h
#define _ROS_command2ros_Digger_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace command2ros
{

  class Digger : public ros::Msg
  {
    public:
      typedef int32_t _raise_type;
      _raise_type raise;

    Digger():
      raise(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_raise;
      u_raise.real = this->raise;
      *(outbuffer + offset + 0) = (u_raise.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_raise.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_raise.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_raise.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->raise);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_raise;
      u_raise.base = 0;
      u_raise.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_raise.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_raise.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_raise.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->raise = u_raise.real;
      offset += sizeof(this->raise);
     return offset;
    }

    const char * getType(){ return "command2ros/Digger"; };
    const char * getMD5(){ return "885174295a67242e340a473c1ac4abc0"; };

  };

}
#endif