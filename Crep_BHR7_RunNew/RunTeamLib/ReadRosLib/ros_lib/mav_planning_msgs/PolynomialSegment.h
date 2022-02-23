#ifndef _ROS_mav_planning_msgs_PolynomialSegment_h
#define _ROS_mav_planning_msgs_PolynomialSegment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace mav_planning_msgs
{

  class PolynomialSegment : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _num_coeffs_type;
      _num_coeffs_type num_coeffs;
      typedef ros::Duration _segment_time_type;
      _segment_time_type segment_time;
      uint32_t x_length;
      typedef double _x_type;
      _x_type st_x;
      _x_type * x;
      uint32_t y_length;
      typedef double _y_type;
      _y_type st_y;
      _y_type * y;
      uint32_t z_length;
      typedef double _z_type;
      _z_type st_z;
      _z_type * z;
      uint32_t rx_length;
      typedef double _rx_type;
      _rx_type st_rx;
      _rx_type * rx;
      uint32_t ry_length;
      typedef double _ry_type;
      _ry_type st_ry;
      _ry_type * ry;
      uint32_t rz_length;
      typedef double _rz_type;
      _rz_type st_rz;
      _rz_type * rz;
      uint32_t yaw_length;
      typedef double _yaw_type;
      _yaw_type st_yaw;
      _yaw_type * yaw;

    PolynomialSegment():
      header(),
      num_coeffs(0),
      segment_time(),
      x_length(0), x(NULL),
      y_length(0), y(NULL),
      z_length(0), z(NULL),
      rx_length(0), rx(NULL),
      ry_length(0), ry(NULL),
      rz_length(0), rz(NULL),
      yaw_length(0), yaw(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_num_coeffs;
      u_num_coeffs.real = this->num_coeffs;
      *(outbuffer + offset + 0) = (u_num_coeffs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_coeffs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_coeffs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_coeffs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_coeffs);
      *(outbuffer + offset + 0) = (this->segment_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segment_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segment_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segment_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_time.sec);
      *(outbuffer + offset + 0) = (this->segment_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segment_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segment_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segment_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_time.nsec);
      *(outbuffer + offset + 0) = (this->x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_length);
      for( uint32_t i = 0; i < x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      *(outbuffer + offset + 0) = (this->y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_length);
      for( uint32_t i = 0; i < y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      *(outbuffer + offset + 0) = (this->z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_length);
      for( uint32_t i = 0; i < z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_zi;
      u_zi.real = this->z[i];
      *(outbuffer + offset + 0) = (u_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_zi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_zi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_zi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_zi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->z[i]);
      }
      *(outbuffer + offset + 0) = (this->rx_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rx_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rx_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rx_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rx_length);
      for( uint32_t i = 0; i < rx_length; i++){
      union {
        double real;
        uint64_t base;
      } u_rxi;
      u_rxi.real = this->rx[i];
      *(outbuffer + offset + 0) = (u_rxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rxi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rxi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rxi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rxi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rxi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rx[i]);
      }
      *(outbuffer + offset + 0) = (this->ry_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ry_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ry_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ry_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ry_length);
      for( uint32_t i = 0; i < ry_length; i++){
      union {
        double real;
        uint64_t base;
      } u_ryi;
      u_ryi.real = this->ry[i];
      *(outbuffer + offset + 0) = (u_ryi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ryi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ryi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ryi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ryi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ryi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ryi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ryi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ry[i]);
      }
      *(outbuffer + offset + 0) = (this->rz_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rz_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rz_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rz_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rz_length);
      for( uint32_t i = 0; i < rz_length; i++){
      union {
        double real;
        uint64_t base;
      } u_rzi;
      u_rzi.real = this->rz[i];
      *(outbuffer + offset + 0) = (u_rzi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rzi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rzi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rzi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rzi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rzi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rzi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rzi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rz[i]);
      }
      *(outbuffer + offset + 0) = (this->yaw_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_length);
      for( uint32_t i = 0; i < yaw_length; i++){
      union {
        double real;
        uint64_t base;
      } u_yawi;
      u_yawi.real = this->yaw[i];
      *(outbuffer + offset + 0) = (u_yawi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yawi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yawi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yawi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yawi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yawi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yawi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yawi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yaw[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_num_coeffs;
      u_num_coeffs.base = 0;
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_coeffs = u_num_coeffs.real;
      offset += sizeof(this->num_coeffs);
      this->segment_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->segment_time.sec);
      this->segment_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->segment_time.nsec);
      uint32_t x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->x_length);
      if(x_lengthT > x_length)
        this->x = (double*)realloc(this->x, x_lengthT * sizeof(double));
      x_length = x_lengthT;
      for( uint32_t i = 0; i < x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_x;
      u_st_x.base = 0;
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_x = u_st_x.real;
      offset += sizeof(this->st_x);
        memcpy( &(this->x[i]), &(this->st_x), sizeof(double));
      }
      uint32_t y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->y_length);
      if(y_lengthT > y_length)
        this->y = (double*)realloc(this->y, y_lengthT * sizeof(double));
      y_length = y_lengthT;
      for( uint32_t i = 0; i < y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_y;
      u_st_y.base = 0;
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_y = u_st_y.real;
      offset += sizeof(this->st_y);
        memcpy( &(this->y[i]), &(this->st_y), sizeof(double));
      }
      uint32_t z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_length);
      if(z_lengthT > z_length)
        this->z = (double*)realloc(this->z, z_lengthT * sizeof(double));
      z_length = z_lengthT;
      for( uint32_t i = 0; i < z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_z;
      u_st_z.base = 0;
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_z = u_st_z.real;
      offset += sizeof(this->st_z);
        memcpy( &(this->z[i]), &(this->st_z), sizeof(double));
      }
      uint32_t rx_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rx_length);
      if(rx_lengthT > rx_length)
        this->rx = (double*)realloc(this->rx, rx_lengthT * sizeof(double));
      rx_length = rx_lengthT;
      for( uint32_t i = 0; i < rx_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_rx;
      u_st_rx.base = 0;
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_rx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_rx = u_st_rx.real;
      offset += sizeof(this->st_rx);
        memcpy( &(this->rx[i]), &(this->st_rx), sizeof(double));
      }
      uint32_t ry_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ry_length);
      if(ry_lengthT > ry_length)
        this->ry = (double*)realloc(this->ry, ry_lengthT * sizeof(double));
      ry_length = ry_lengthT;
      for( uint32_t i = 0; i < ry_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_ry;
      u_st_ry.base = 0;
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_ry.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_ry = u_st_ry.real;
      offset += sizeof(this->st_ry);
        memcpy( &(this->ry[i]), &(this->st_ry), sizeof(double));
      }
      uint32_t rz_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rz_length);
      if(rz_lengthT > rz_length)
        this->rz = (double*)realloc(this->rz, rz_lengthT * sizeof(double));
      rz_length = rz_lengthT;
      for( uint32_t i = 0; i < rz_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_rz;
      u_st_rz.base = 0;
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_rz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_rz = u_st_rz.real;
      offset += sizeof(this->st_rz);
        memcpy( &(this->rz[i]), &(this->st_rz), sizeof(double));
      }
      uint32_t yaw_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_length);
      if(yaw_lengthT > yaw_length)
        this->yaw = (double*)realloc(this->yaw, yaw_lengthT * sizeof(double));
      yaw_length = yaw_lengthT;
      for( uint32_t i = 0; i < yaw_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_yaw;
      u_st_yaw.base = 0;
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_yaw.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_yaw = u_st_yaw.real;
      offset += sizeof(this->st_yaw);
        memcpy( &(this->yaw[i]), &(this->st_yaw), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "mav_planning_msgs/PolynomialSegment"; };
    const char * getMD5(){ return "1bfc920140297f14773c46c1eacc4c1d"; };

  };

}
#endif
