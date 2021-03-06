#ifndef _ROS_asctec_hl_comm_mav_status_h
#define _ROS_asctec_hl_comm_mav_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace asctec_hl_comm
{

  class mav_status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef const char* _flight_mode_ll_type;
      _flight_mode_ll_type flight_mode_ll;
      typedef const char* _state_estimation_type;
      _state_estimation_type state_estimation;
      typedef const char* _position_control_type;
      _position_control_type position_control;
      typedef bool _serial_interface_enabled_type;
      _serial_interface_enabled_type serial_interface_enabled;
      typedef bool _serial_interface_active_type;
      _serial_interface_active_type serial_interface_active;
      typedef float _flight_time_type;
      _flight_time_type flight_time;
      typedef float _cpu_load_type;
      _cpu_load_type cpu_load;
      typedef const char* _motor_status_type;
      _motor_status_type motor_status;
      typedef const char* _gps_status_type;
      _gps_status_type gps_status;
      typedef int32_t _gps_num_satellites_type;
      _gps_num_satellites_type gps_num_satellites;
      typedef int32_t _debug1_type;
      _debug1_type debug1;
      typedef int32_t _debug2_type;
      _debug2_type debug2;
      typedef bool _have_SSDK_parameters_type;
      _have_SSDK_parameters_type have_SSDK_parameters;
      typedef uint32_t _tx_packets_type;
      _tx_packets_type tx_packets;
      typedef uint32_t _tx_packets_good_type;
      _tx_packets_good_type tx_packets_good;
      typedef uint32_t _rx_packets_type;
      _rx_packets_type rx_packets;
      typedef uint32_t _rx_packets_good_type;
      _rx_packets_good_type rx_packets_good;
      typedef float _timesync_offset_type;
      _timesync_offset_type timesync_offset;

    mav_status():
      header(),
      battery_voltage(0),
      flight_mode_ll(""),
      state_estimation(""),
      position_control(""),
      serial_interface_enabled(0),
      serial_interface_active(0),
      flight_time(0),
      cpu_load(0),
      motor_status(""),
      gps_status(""),
      gps_num_satellites(0),
      debug1(0),
      debug2(0),
      have_SSDK_parameters(0),
      tx_packets(0),
      tx_packets_good(0),
      rx_packets(0),
      rx_packets_good(0),
      timesync_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      uint32_t length_flight_mode_ll = strlen(this->flight_mode_ll);
      varToArr(outbuffer + offset, length_flight_mode_ll);
      offset += 4;
      memcpy(outbuffer + offset, this->flight_mode_ll, length_flight_mode_ll);
      offset += length_flight_mode_ll;
      uint32_t length_state_estimation = strlen(this->state_estimation);
      varToArr(outbuffer + offset, length_state_estimation);
      offset += 4;
      memcpy(outbuffer + offset, this->state_estimation, length_state_estimation);
      offset += length_state_estimation;
      uint32_t length_position_control = strlen(this->position_control);
      varToArr(outbuffer + offset, length_position_control);
      offset += 4;
      memcpy(outbuffer + offset, this->position_control, length_position_control);
      offset += length_position_control;
      union {
        bool real;
        uint8_t base;
      } u_serial_interface_enabled;
      u_serial_interface_enabled.real = this->serial_interface_enabled;
      *(outbuffer + offset + 0) = (u_serial_interface_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->serial_interface_enabled);
      union {
        bool real;
        uint8_t base;
      } u_serial_interface_active;
      u_serial_interface_active.real = this->serial_interface_active;
      *(outbuffer + offset + 0) = (u_serial_interface_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->serial_interface_active);
      union {
        float real;
        uint32_t base;
      } u_flight_time;
      u_flight_time.real = this->flight_time;
      *(outbuffer + offset + 0) = (u_flight_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flight_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flight_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flight_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flight_time);
      union {
        float real;
        uint32_t base;
      } u_cpu_load;
      u_cpu_load.real = this->cpu_load;
      *(outbuffer + offset + 0) = (u_cpu_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_load);
      uint32_t length_motor_status = strlen(this->motor_status);
      varToArr(outbuffer + offset, length_motor_status);
      offset += 4;
      memcpy(outbuffer + offset, this->motor_status, length_motor_status);
      offset += length_motor_status;
      uint32_t length_gps_status = strlen(this->gps_status);
      varToArr(outbuffer + offset, length_gps_status);
      offset += 4;
      memcpy(outbuffer + offset, this->gps_status, length_gps_status);
      offset += length_gps_status;
      union {
        int32_t real;
        uint32_t base;
      } u_gps_num_satellites;
      u_gps_num_satellites.real = this->gps_num_satellites;
      *(outbuffer + offset + 0) = (u_gps_num_satellites.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gps_num_satellites.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gps_num_satellites.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gps_num_satellites.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gps_num_satellites);
      union {
        int32_t real;
        uint32_t base;
      } u_debug1;
      u_debug1.real = this->debug1;
      *(outbuffer + offset + 0) = (u_debug1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_debug1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_debug1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_debug1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->debug1);
      union {
        int32_t real;
        uint32_t base;
      } u_debug2;
      u_debug2.real = this->debug2;
      *(outbuffer + offset + 0) = (u_debug2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_debug2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_debug2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_debug2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->debug2);
      union {
        bool real;
        uint8_t base;
      } u_have_SSDK_parameters;
      u_have_SSDK_parameters.real = this->have_SSDK_parameters;
      *(outbuffer + offset + 0) = (u_have_SSDK_parameters.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->have_SSDK_parameters);
      *(outbuffer + offset + 0) = (this->tx_packets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tx_packets >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tx_packets >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tx_packets >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tx_packets);
      *(outbuffer + offset + 0) = (this->tx_packets_good >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tx_packets_good >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tx_packets_good >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tx_packets_good >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tx_packets_good);
      *(outbuffer + offset + 0) = (this->rx_packets >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rx_packets >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rx_packets >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rx_packets >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rx_packets);
      *(outbuffer + offset + 0) = (this->rx_packets_good >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rx_packets_good >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rx_packets_good >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rx_packets_good >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rx_packets_good);
      union {
        float real;
        uint32_t base;
      } u_timesync_offset;
      u_timesync_offset.real = this->timesync_offset;
      *(outbuffer + offset + 0) = (u_timesync_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timesync_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timesync_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timesync_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timesync_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      uint32_t length_flight_mode_ll;
      arrToVar(length_flight_mode_ll, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_flight_mode_ll; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_flight_mode_ll-1]=0;
      this->flight_mode_ll = (char *)(inbuffer + offset-1);
      offset += length_flight_mode_ll;
      uint32_t length_state_estimation;
      arrToVar(length_state_estimation, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_estimation; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_estimation-1]=0;
      this->state_estimation = (char *)(inbuffer + offset-1);
      offset += length_state_estimation;
      uint32_t length_position_control;
      arrToVar(length_position_control, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_position_control; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_position_control-1]=0;
      this->position_control = (char *)(inbuffer + offset-1);
      offset += length_position_control;
      union {
        bool real;
        uint8_t base;
      } u_serial_interface_enabled;
      u_serial_interface_enabled.base = 0;
      u_serial_interface_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->serial_interface_enabled = u_serial_interface_enabled.real;
      offset += sizeof(this->serial_interface_enabled);
      union {
        bool real;
        uint8_t base;
      } u_serial_interface_active;
      u_serial_interface_active.base = 0;
      u_serial_interface_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->serial_interface_active = u_serial_interface_active.real;
      offset += sizeof(this->serial_interface_active);
      union {
        float real;
        uint32_t base;
      } u_flight_time;
      u_flight_time.base = 0;
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flight_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->flight_time = u_flight_time.real;
      offset += sizeof(this->flight_time);
      union {
        float real;
        uint32_t base;
      } u_cpu_load;
      u_cpu_load.base = 0;
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_load = u_cpu_load.real;
      offset += sizeof(this->cpu_load);
      uint32_t length_motor_status;
      arrToVar(length_motor_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor_status-1]=0;
      this->motor_status = (char *)(inbuffer + offset-1);
      offset += length_motor_status;
      uint32_t length_gps_status;
      arrToVar(length_gps_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_gps_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_gps_status-1]=0;
      this->gps_status = (char *)(inbuffer + offset-1);
      offset += length_gps_status;
      union {
        int32_t real;
        uint32_t base;
      } u_gps_num_satellites;
      u_gps_num_satellites.base = 0;
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gps_num_satellites.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gps_num_satellites = u_gps_num_satellites.real;
      offset += sizeof(this->gps_num_satellites);
      union {
        int32_t real;
        uint32_t base;
      } u_debug1;
      u_debug1.base = 0;
      u_debug1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_debug1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_debug1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_debug1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->debug1 = u_debug1.real;
      offset += sizeof(this->debug1);
      union {
        int32_t real;
        uint32_t base;
      } u_debug2;
      u_debug2.base = 0;
      u_debug2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_debug2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_debug2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_debug2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->debug2 = u_debug2.real;
      offset += sizeof(this->debug2);
      union {
        bool real;
        uint8_t base;
      } u_have_SSDK_parameters;
      u_have_SSDK_parameters.base = 0;
      u_have_SSDK_parameters.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->have_SSDK_parameters = u_have_SSDK_parameters.real;
      offset += sizeof(this->have_SSDK_parameters);
      this->tx_packets =  ((uint32_t) (*(inbuffer + offset)));
      this->tx_packets |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tx_packets |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tx_packets |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tx_packets);
      this->tx_packets_good =  ((uint32_t) (*(inbuffer + offset)));
      this->tx_packets_good |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tx_packets_good |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tx_packets_good |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tx_packets_good);
      this->rx_packets =  ((uint32_t) (*(inbuffer + offset)));
      this->rx_packets |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rx_packets |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->rx_packets |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->rx_packets);
      this->rx_packets_good =  ((uint32_t) (*(inbuffer + offset)));
      this->rx_packets_good |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rx_packets_good |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->rx_packets_good |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->rx_packets_good);
      union {
        float real;
        uint32_t base;
      } u_timesync_offset;
      u_timesync_offset.base = 0;
      u_timesync_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timesync_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timesync_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timesync_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timesync_offset = u_timesync_offset.real;
      offset += sizeof(this->timesync_offset);
     return offset;
    }

    const char * getType(){ return "asctec_hl_comm/mav_status"; };
    const char * getMD5(){ return "f975cbdf223868931f194323c62d7be5"; };

  };

}
#endif
