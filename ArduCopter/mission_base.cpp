#if MISSION_MANAGE == ENABLED
#include "Copter.h"
#include "mission_base.h"

void Mission_base::set_postion(int32_t lat, int32_t lon, int32_t alt, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false) //yaw_ignore==true do not consider yaw
{
    Vector3f pos_ned;
    // sanity check location
    if (!check_latlng(lat, lon))
        return;
    Location loc;
    loc.lat = lat;
    loc.lng = lon;
    loc.alt = alt * 100;
    loc.flags.relative_alt = true;
    loc.flags.terrain_alt = false;
    pos_ned = copter.pv_location_to_vector(loc);
    // prepare yaw
    float yaw_cd = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_cds = 0.0f;
    if (!yaw_ignore)
    {
        yaw_cd = ToDeg(yaw) * 100.0f;
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
    copter.guided_set_destination(pos_ned, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}
void Mission_base::set_velocity(float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false)
{
    // prepare yaw
    float yaw_cd = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_cds = 0.0f;
    if (!yaw_ignore)
    {
        yaw_cd = ToDeg(yaw) * 100.0f;
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
    copter.guided_set_velocity(Vector3f(vx * 100.0f, vy * 100.0f, -vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}

void Mission_base::set_posvel(int32_t lat, int32_t lon, int32_t alt, float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false) //yaw_ignore==1 do not consider yaw
{
    Vector3f pos_ned;
    // sanity check location
    if (!check_latlng(lat, lon))
        return;
    Location loc;
    loc.lat = lat;
    loc.lng = lon;
    loc.alt = alt * 100;
    loc.flags.relative_alt = true;
    loc.flags.terrain_alt = false;
    pos_ned = copter.pv_location_to_vector(loc);
    // prepare yaw
    float yaw_cd = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_cds = 0.0f;
    if (!yaw_ignore)
    {
        yaw_cd = ToDeg(yaw) * 100.0f;
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
    copter.guided_set_destination_posvel(pos_ned, Vector3f(vx * 100.0f, vy * 100.0f, -vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}

void Mission_pigeon::set_para(uint8_t* p_10, float* p_15)
{

}
void Mission_pigeon::init()
{

    init_flag=true;
}
void Mission_pigeon::run()
{
    status_int[0]++;

}

#endif
