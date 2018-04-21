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
      typedef int32_t _serialID_type;
      _serialID_type serialID;
      typedef bool _manual_type;
      _manual_type manual;
      typedef int32_t _manualDrive_type;
      _manualDrive_type manualDrive;
      typedef int32_t _manualTurn_type;
      _manualTurn_type manualTurn;
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
      typedef bool _pause_type;
      _pause_type pause;
      typedef bool _cancel_type;
      _cancel_type cancel;

    MovementCommand():
      serialID(0),
      manual(0),
      manualDrive(0),
      manualTurn(0),
      driveDist(0),
      turn(0),
      dig(0),
      dump(0),
      packin(0),
      eStop(0),
      pause(0),
      cancel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      union {
        bool real;
        uint8_t base;
      } u_manual;
      u_manual.real = this->manual;
      *(outbuffer + offset + 0) = (u_manual.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->manual);
      union {
        int32_t real;
        uint32_t base;
      } u_manualDrive;
      u_manualDrive.real = this->manualDrive;
      *(outbuffer + offset + 0) = (u_manualDrive.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_manualDrive.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_manualDrive.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_manualDrive.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->manualDrive);
      union {
        int32_t real;
        uint32_t base;
      } u_manualTurn;
      u_manualTurn.real = this->manualTurn;
      *(outbuffer + offset + 0) = (u_manualTurn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_manualTurn.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_manualTurn.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_manualTurn.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->manualTurn);
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
      } u_pause;
      u_pause.real = this->pause;
      *(outbuffer + offset + 0) = (u_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pause);
      union {
        bool real;
        uint8_t base;
      } u_cancel;
      u_cancel.real = this->cancel;
      *(outbuffer + offset + 0) = (u_cancel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cancel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      union {
        bool real;
        uint8_t base;
      } u_manual;
      u_manual.base = 0;
      u_manual.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->manual = u_manual.real;
      offset += sizeof(this->manual);
      union {
        int32_t real;
        uint32_t base;
      } u_manualDrive;
      u_manualDrive.base = 0;
      u_manualDrive.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_manualDrive.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_manualDrive.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_manualDrive.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->manualDrive = u_manualDrive.real;
      offset += sizeof(this->manualDrive);
      union {
        int32_t real;
        uint32_t base;
      } u_manualTurn;
      u_manualTurn.base = 0;
      u_manualTurn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_manualTurn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_manualTurn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_manualTurn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->manualTurn = u_manualTurn.real;
      offset += sizeof(this->manualTurn);
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
      } u_pause;
      u_pause.base = 0;
      u_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pause = u_pause.real;
      offset += sizeof(this->pause);
      union {
        bool real;
        uint8_t base;
      } u_cancel;
      u_cancel.base = 0;
      u_cancel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cancel = u_cancel.real;
      offset += sizeof(this->cancel);
     return offset;
    }

    const char * getType(){ return "command2ros/MovementCommand"; };
    const char * getMD5(){ return "313c15045e6d52239964373121ab45c4"; };

  };

}
#endif