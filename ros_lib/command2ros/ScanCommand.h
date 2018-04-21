#ifndef _ROS_command2ros_ScanCommand_h
#define _ROS_command2ros_ScanCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace command2ros
{

  class ScanCommand : public ros::Msg
  {
    public:
      typedef bool _scan_type;
      _scan_type scan;
      typedef int32_t _serialID_type;
      _serialID_type serialID;

    ScanCommand():
      scan(0),
      serialID(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_scan;
      u_scan.real = this->scan;
      *(outbuffer + offset + 0) = (u_scan.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->scan);
      union {
        int32_t real;
        uint32_t base;
      } u_serialID;
      u_serialID.real = this->serialID;
      *(outbuffer + offset + 0) = (u_serialID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_serialID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_serialID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_serialID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->serialID);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_scan;
      u_scan.base = 0;
      u_scan.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->scan = u_scan.real;
      offset += sizeof(this->scan);
      union {
        int32_t real;
        uint32_t base;
      } u_serialID;
      u_serialID.base = 0;
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_serialID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->serialID = u_serialID.real;
      offset += sizeof(this->serialID);
     return offset;
    }

    const char * getType(){ return "command2ros/ScanCommand"; };
    const char * getMD5(){ return "81b09f3ef0899a3b0506cdfc2d853e80"; };

  };

}
#endif