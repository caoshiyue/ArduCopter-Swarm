#pragma once
#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "publisher.h"

namespace CUSTOM_MISSION
{
#ifndef this_id
#define this_id mavlink_system.sysid
#endif 
class Mission_base
{
public:
  Mission_base() {}
  ~Mission_base() {}

  static uint8_t counter;
  static publisher pub;
  static subscriber sub;
  static Location my_Pxyz; //int32 alt in cm positive up
  static float my_vx ;//my velocity m/s
  static float my_vy ;
  static float my_vz ;//positive up m/s
  static float my_yaw;
  static void update_mypvy(); 
  static Mav_status get_mav(uint8_t);
  static float limit_av(float, float);
  bool init_flag = false;
  void send_mission_status(mavlink_channel_t chan, uint8_t interval, const char *fmt, ...); // maybe can use it as a heartbeat between flock, scan nods and get id
  /// alt vz positive up
  void set_postion(int32_t lat, int32_t lon, float alt, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);
  void set_velocity(float _vx, float _vy, float _vz, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);
  void set_posvel(int32_t lat, int32_t lon, float alt, float _vx, float _vy, float _vz, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);
  void set_position_target_global(mavlink_set_position_target_global_int_t &packet);
  
  virtual void set_para(uint8_t *, float *) = 0;
  virtual void init() = 0;
  virtual void run() = 0;
};
/*
NOTE:
int32 alt in cm
float yaw in degree
sin() atan()... in rad
*/

//-----------------------------------Mission zero ----------------------------------------------
class Mission_default_swarm : public Mission_base
{
public:
  Mission_default_swarm() {}
  ~Mission_default_swarm() {}
  void set_para(uint8_t *, float *);
  void init();
  void run();

private:                    //parameters and algorithm
  float disx = 0;           // offset of my position if leader is origin in NED coordinate  P
  float disy = 0;           //      P
  float vm = 3.0;           //max velocity           P
  float proportion_v = 0.2; //proportion control gain of velocity    P
  uint8_t up;               //  leader id     P
};

//-----------------------------------Mission one----------------------------------------------
class Mission_pigeon : public Mission_base
{
public:
  Mission_pigeon() {}
  ~Mission_pigeon() {}
  void set_para(uint8_t *, float *);
  void init();
  void run();

private: //parameters and algorithm
  float dis[4] = {0};
  float disx[4] = {0}; // NED coordinate     //      P
  float disy[4] = {0}; //      P
  uint8_t upper[4] = {0};

  float K = 0.01;
  uint8_t kp = 4; //      P
  uint8_t kv = 2; //      P
  float w1 = 2.0; //      P
  uint8_t kh = 3; //      P
  float w2 = 2.0; //      P
  uint8_t m = 1;
  float tv = 0.1;
  float vm = 2.0;
  float am = 3.0;
  float k2x1 = 0.4; //      P
  float k2x2 = 0.4; //      P
  uint8_t kxy = 150;

  float force_target(bool x);
  float force_form(uint8_t i, bool x);
  float repulsion(float dis, float range, float sudden, bool x);
  float u_x();
  float u_y();
  float u_z();
  void upload_var();
  void get_upper();
};


} //namespace
#endif
