#pragma once
#if MISSION_MANAGE == ENABLED
#include "Copter.h"

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
    uint8_t status_int[3]={0};
    float status_float[3]={0.0};
    Location Pxyz;  //int32 alt in cm
    float vx = 0;    
    float vy = 0;
    float vz = 0;
    float yaw = 0;
    void set_postion(int32_t lat, int32_t lon, float alt, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false); 
    void set_velocity(float _vx, float _vy, float _vz, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false);//up vz<0
    void set_posvel(int32_t lat, int32_t lon, float alt, float _vx, float _vy, float _vz, float _yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false);
    void upload_pvy();
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
    uint8_t kv = 2;  //      P
    float w1 = 2.0;  //      P
    uint8_t kh = 3;  //      P
    float w2 = 2.0;  //      P
    uint8_t m = 1;
    float tv = 0.1;
    float vm = 2.0;
    float am = 3.0;
    float k2x1 = 0.4; //      P
    float k2x2 = 0.4; //      P
    uint8_t kxy = 150;

    float force_target(bool x);
    float force_form(uint8_t i, bool x);
    float repulsion(float dis,float range, float sudden,bool x);
    float u_x();
    float u_y();
    float u_z();
    void upload_var();
    void get_upper();
};
#endif  //this_id
#endif
