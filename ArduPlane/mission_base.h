#pragma once
#if MISSION_MANAGE == ENABLED
#include "Plane.h"

#ifndef this_id
#define this_id mavlink_system.sysid

class Mavlist
{
    public:
    uint32_t time_boot_ms=0; /*< Timestamp (milliseconds since system boot)*/
    int32_t altitude=0; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
    Location Pxyz; /*int32  alt in cm */    
    float vx=0; /*< Ground X Speed (Latitude, positive north), expressed as m/s * */
    float vy=0; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * */
    float vz=0; /*< Ground Z Speed (Altitude, positive up), expressed as m/s * */
    float yaw=0; /*< Vehicle heading (yaw angle) in degrees , 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    void update(mavlink_global_position_int_t packet);
};
/*
extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location &loc, float ofs_north, float ofs_east);

return the distance in meters as a N/E vector from loc1 to loc2     loc2-loc1 in Meter
Vector2f    location_diff(const struct Location &loc1, const struct Location &loc2);    
*/


class Mission_base
{
    public:
    Mission_base(){}
    ~Mission_base(){}
    bool init_flag=false;
    Mavlist mav[MAX_FLOCK_NUM]; //  1~x correspond with sysid
    uint8_t status_int[3]={0};// int status return to gcs
    float status_float[3]={0.0};// float status return to gcs
    Location Pxyz;  //int32 alt in cm
    float vx = 0;    //my velocity
    float vy = 0;
    float vz = 0;
    float yaw = 0;
    void set_attitude();
    void set_posvel(int32_t lat, int32_t lon, float alt, float _vx, float _vy, float _vz, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false);
    void upload_pvy();  // send my pos and vel to others
    float limit_av(float v,float limit);  
    virtual void set_para(uint8_t* ,float* )=0;
    virtual void init()=0;
    virtual void run()=0;
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

  private: //parameters and algorithm
    float disx = 0; // offset of my position if leader is origin in NED coordinate  P
    float disy = 0; //      P
    float vm = 3.0; //max velocity           P
    float proportion_v=0.2;//proportion control gain of velocity    P
    uint8_t up; //  leader id     P
};


//-----------------------------------Mission one----------------------------------------------

#endif  //this_id
#endif
