#pragma once
#if MISSION_MANAGE == ENABLED
#include "Copter.h"

class Mission_base
{
    public:
    bool init_flag=false;
    uint8_t status_int[3]={0};
    float status_float[3]={0.0};
    void set_postion(int32_t lat, int32_t lon, int32_t alt, float yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore); 
    void set_velocity(float vx, float vy, float vz, float yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore);
    void set_posvel(int32_t lat, int32_t lon, int32_t alt, float vx, float vy, float vz, float yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore);
    virtual void set_para(uint8_t* ,float* );
    virtual void init();
    virtual void run();
};
class Mavlist
{
    public:
    uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
    int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
    int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
    int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
    int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
    int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
    int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
    int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
    uint16_t yaw_100; /*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/


}
class Mission_pigeon :public Mission_base //mission 1: pigeon_formation
{
    public:
      void set_para(uint8_t *, float *);
      void init();      
      void run();

    private:    //parameters and algorithm


};

#endif
