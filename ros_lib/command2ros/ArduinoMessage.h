#ifndef _ROS_command2ros_ArduinoMessage_h
#define _ROS_command2ros_ArduinoMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace command2ros
{

  class ArduinoMessage : public ros::Msg
  {
    public:
      typedef bool _ready_type;
      _ready_type ready;
      typedef int32_t _serialID_type;
      _serialID_type serialID;
      typedef const char* _progress_type;
      _progress_type progress;
      typedef bool _errorDriving_type;
      _errorDriving_type errorDriving;
      typedef bool _errorDigging_type;
      _errorDigging_type errorDigging;
      typedef bool _errorDumping_type;
      _errorDumping_type errorDumping;
      typedef bool _errorTurning_type;
      _errorTurning_type errorTurning;
      typedef int32_t _messageID_type;
      _messageID_type messageID;

    ArduinoMessage():
      ready(0),
      serialID(0),
      progress(""),
      errorDriving(0),
      errorDigging(0),
      errorDumping(0),
      errorTurning(0),
      messageID(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ready;
      u_ready.real = this->ready;
      *(outbuffer + offset + 0) = (u_ready.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ready);
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
      uint32_t length_progress = strlen(this->progress);
      varToArr(outbuffer + offset, length_progress);
      offset += 4;
      memcpy(outbuffer + offset, this->progress, length_progress);
      offset += length_progress;
      union {
        bool real;
        uint8_t base;
      } u_errorDriving;
      u_errorDriving.real = this->errorDriving;
      *(outbuffer + offset + 0) = (u_errorDriving.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errorDriving);
      union {
        bool real;
        uint8_t base;
      } u_errorDigging;
      u_errorDigging.real = this->errorDigging;
      *(outbuffer + offset + 0) = (u_errorDigging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errorDigging);
      union {
        bool real;
        uint8_t base;
      } u_errorDumping;
      u_errorDumping.real = this->errorDumping;
      *(outbuffer + offset + 0) = (u_errorDumping.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errorDumping);
      union {
        bool real;
        uint8_t base;
      } u_errorTurning;
      u_errorTurning.real = this->errorTurning;
      *(outbuffer + offset + 0) = (u_errorTurning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->errorTurning);
      union {
        int32_t real;
        uint32_t base;
      } u_messageID;
      u_messageID.real = this->messageID;
      *(outbuffer + offset + 0) = (u_messageID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_messageID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_messageID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_messageID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->messageID);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ready;
      u_ready.base = 0;
      u_ready.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ready = u_ready.real;
      offset += sizeof(this->ready);
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
      uint32_t length_progress;
      arrToVar(length_progress, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_progress; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_progress-1]=0;
      this->progress = (char *)(inbuffer + offset-1);
      offset += length_progress;
      union {
        bool real;
        uint8_t base;
      } u_errorDriving;
      u_errorDriving.base = 0;
      u_errorDriving.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->errorDriving = u_errorDriving.real;
      offset += sizeof(this->errorDriving);
      union {
        bool real;
        uint8_t base;
      } u_errorDigging;
      u_errorDigging.base = 0;
      u_errorDigging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->errorDigging = u_errorDigging.real;
      offset += sizeof(this->errorDigging);
      union {
        bool real;
        uint8_t base;
      } u_errorDumping;
      u_errorDumping.base = 0;
      u_errorDumping.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->errorDumping = u_errorDumping.real;
      offset += sizeof(this->errorDumping);
      union {
        bool real;
        uint8_t base;
      } u_errorTurning;
      u_errorTurning.base = 0;
      u_errorTurning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->errorTurning = u_errorTurning.real;
      offset += sizeof(this->errorTurning);
      union {
        int32_t real;
        uint32_t base;
      } u_messageID;
      u_messageID.base = 0;
      u_messageID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_messageID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_messageID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_messageID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->messageID = u_messageID.real;
      offset += sizeof(this->messageID);
     return offset;
    }

    const char * getType(){ return "command2ros/ArduinoMessage"; };
    const char * getMD5(){ return "a7d9c2f1772c7f2f2538e66a8339c845"; };

  };

}
#endif