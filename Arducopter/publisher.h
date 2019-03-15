#pragma once
#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include <list> 
#include "Map.h"

namespace CUSTOM_MISSION
{
#ifndef this_id
#define this_id mavlink_system.sysid
#endif 	
class Mav_status
{
public:
  uint32_t time_boot_ms = 0; /*< Timestamp (milliseconds since system boot)*/
  int32_t altitude = 0;      /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
  Location Pxyz;             /*int32  alt in cm */
  float vx = 0;              /*< Ground X Speed (Latitude, positive north), expressed as m/s * */
  float vy = 0;              /*< Ground Y Speed (Longitude, positive east), expressed as m/s * */
  float vz = 0;              /*< Ground Z Speed (Altitude, positive up), expressed as m/s * */
  float yaw = 0;             /*< Vehicle heading (yaw angle) in degrees , 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
  void update(mavlink_global_position_int_t packet);
};

class publisher
{
  public:
	void notify();
	void handle_regist_msg(uint8_t, bool); //register or cancel
  private:
  bool find(uint8_t);
  std::list<uint8_t> mav_sub;				   // it will be multimap, if diverse msg are required
};

class subscriber
{
  public:
	void regist(uint8_t, bool);
	void handle_notify_msg(uint8_t,mavlink_global_position_int_t );
	Map<uint8_t, Mav_status> mav_map;
};
const mavlink_channel_t gcs_chan=mavlink_channel_t(2);
const mavlink_channel_t broad_chan=mavlink_channel_t(5);
mavlink_channel_t get_mav_chan(uint8_t);

}
#endif