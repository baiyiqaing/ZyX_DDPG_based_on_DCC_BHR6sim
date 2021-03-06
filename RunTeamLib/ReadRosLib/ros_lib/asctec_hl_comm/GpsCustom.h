#ifndef _ROS_asctec_hl_comm_GpsCustom_h
#define _ROS_asctec_hl_comm_GpsCustom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatStatus.h"

namespace asctec_hl_comm
{

  class GpsCustom : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::NavSatStatus _status_type;
      _status_type status;
      typedef double _latitude_type;
      _latitude_type latitude;
      typedef double _longitude_type;
      _longitude_type longitude;
      typedef double _altitude_type;
      _altitude_type altitude;
      double position_covariance[9];
      typedef uint8_t _position_covariance_type_type;
      _position_covariance_type_type position_covariance_type;
      typedef double _pressure_height_type;
      _pressure_height_type pressure_height;
      typedef double _velocity_x_type;
      _velocity_x_type velocity_x;
      typedef double _velocity_y_type;
      _velocity_y_type velocity_y;
      double velocity_covariance[4];

    GpsCustom():
      header(),
      status(),
      latitude(0),
      longitude(0),
      altitude(0),
      position_covariance(),
      position_covariance_type(0),
      pressure_height(0),
      velocity_x(0),
      velocity_y(0),
      velocity_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_latitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_latitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_latitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_latitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_longitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_longitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_longitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_longitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_altitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_altitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_altitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_altitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->altitude);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_position_covariancei;
      u_position_covariancei.real = this->position_covariance[i];
      *(outbuffer + offset + 0) = (u_position_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      union {
        double real;
        uint64_t base;
      } u_pressure_height;
      u_pressure_height.real = this->pressure_height;
      *(outbuffer + offset + 0) = (u_pressure_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure_height.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pressure_height.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pressure_height.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pressure_height.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pressure_height.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pressure_height);
      union {
        double real;
        uint64_t base;
      } u_velocity_x;
      u_velocity_x.real = this->velocity_x;
      *(outbuffer + offset + 0) = (u_velocity_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_x);
      union {
        double real;
        uint64_t base;
      } u_velocity_y;
      u_velocity_y.real = this->velocity_y;
      *(outbuffer + offset + 0) = (u_velocity_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_y);
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_covariancei;
      u_velocity_covariancei.real = this->velocity_covariance[i];
      *(outbuffer + offset + 0) = (u_velocity_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_position_covariancei;
      u_position_covariancei.base = 0;
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position_covariance[i] = u_position_covariancei.real;
      offset += sizeof(this->position_covariance[i]);
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
      union {
        double real;
        uint64_t base;
      } u_pressure_height;
      u_pressure_height.base = 0;
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pressure_height.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pressure_height = u_pressure_height.real;
      offset += sizeof(this->pressure_height);
      union {
        double real;
        uint64_t base;
      } u_velocity_x;
      u_velocity_x.base = 0;
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity_x = u_velocity_x.real;
      offset += sizeof(this->velocity_x);
      union {
        double real;
        uint64_t base;
      } u_velocity_y;
      u_velocity_y.base = 0;
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity_y = u_velocity_y.real;
      offset += sizeof(this->velocity_y);
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_covariancei;
      u_velocity_covariancei.base = 0;
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_velocity_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->velocity_covariance[i] = u_velocity_covariancei.real;
      offset += sizeof(this->velocity_covariance[i]);
      }
     return offset;
    }

    const char * getType(){ return "asctec_hl_comm/GpsCustom"; };
    const char * getMD5(){ return "ea845c87e3fc5ff92a4bebb639327746"; };

  };

}
#endif
