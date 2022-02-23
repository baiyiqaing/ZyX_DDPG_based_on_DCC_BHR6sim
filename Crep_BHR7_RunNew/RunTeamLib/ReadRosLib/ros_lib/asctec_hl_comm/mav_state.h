#ifndef _ROS_asctec_hl_comm_mav_state_h
#define _ROS_asctec_hl_comm_mav_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace asctec_hl_comm
{

  class mav_state : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Vector3 _velocity_type;
      _velocity_type velocity;

    mav_state():
      header(),
      pose(),
      velocity()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "asctec_hl_comm/mav_state"; };
    const char * getMD5(){ return "e034f695cee32efa98d5cb7e960000e2"; };

  };

}
#endif
