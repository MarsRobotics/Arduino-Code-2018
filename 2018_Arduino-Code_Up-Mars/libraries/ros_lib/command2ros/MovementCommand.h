#ifndef _ROS_command2ros_MovementCommand_h
#define _ROS_command2ros_MovementCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace command2ros
{

  class MovementCommand : public ros::Msg
  {
    public:
      typedef float _driveDist_type;
      _driveDist_type driveDist;
      typedef int32_t _turn_type;
      _turn_type turn;
      typedef bool _dig_type;
      _dig_type dig;
      typedef bool _dump_type;
      _dump_type dump;
      typedef bool _packin_type;
      _packin_type packin;
      typedef bool _eStop_type;
      _eStop_type eStop;
      typedef bool _stop_type;
      _stop_type stop;

    MovementCommand():
      driveDist(0),
      turn(0),
      dig(0),
      dump(0),
      packin(0),
      eStop(0),
      stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_driveDist;
      u_driveDist.real = this->driveDist;
      *(outbuffer + offset + 0) = (u_driveDist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_driveDist.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_driveDist.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_driveDist.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->driveDist);
      union {
        int32_t real;
        uint32_t base;
      } u_turn;
      u_turn.real = this->turn;
      *(outbuffer + offset + 0) = (u_turn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_turn.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_turn.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_turn.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->turn);
      union {
        bool real;
        uint8_t base;
      } u_dig;
      u_dig.real = this->dig;
      *(outbuffer + offset + 0) = (u_dig.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dig);
      union {
        bool real;
        uint8_t base;
      } u_dump;
      u_dump.real = this->dump;
      *(outbuffer + offset + 0) = (u_dump.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dump);
      union {
        bool real;
        uint8_t base;
      } u_packin;
      u_packin.real = this->packin;
      *(outbuffer + offset + 0) = (u_packin.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->packin);
      union {
        bool real;
        uint8_t base;
      } u_eStop;
      u_eStop.real = this->eStop;
      *(outbuffer + offset + 0) = (u_eStop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->eStop);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_driveDist;
      u_driveDist.base = 0;
      u_driveDist.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_driveDist.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_driveDist.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_driveDist.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->driveDist = u_driveDist.real;
      offset += sizeof(this->driveDist);
      union {
        int32_t real;
        uint32_t base;
      } u_turn;
      u_turn.base = 0;
      u_turn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_turn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_turn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_turn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->turn = u_turn.real;
      offset += sizeof(this->turn);
      union {
        bool real;
        uint8_t base;
      } u_dig;
      u_dig.base = 0;
      u_dig.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dig = u_dig.real;
      offset += sizeof(this->dig);
      union {
        bool real;
        uint8_t base;
      } u_dump;
      u_dump.base = 0;
      u_dump.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dump = u_dump.real;
      offset += sizeof(this->dump);
      union {
        bool real;
        uint8_t base;
      } u_packin;
      u_packin.base = 0;
      u_packin.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->packin = u_packin.real;
      offset += sizeof(this->packin);
      union {
        bool real;
        uint8_t base;
      } u_eStop;
      u_eStop.base = 0;
      u_eStop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->eStop = u_eStop.real;
      offset += sizeof(this->eStop);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
     return offset;
    }

    const char * getType(){ return "command2ros/MovementCommand"; };
    const char * getMD5(){ return "6cec864cca46ad9a22762b3b64dcff7b"; };

  };

}
#endif