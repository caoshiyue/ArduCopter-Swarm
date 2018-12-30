#if MISSION_MANAGE == ENABLED
#include "Plane.h"
#include "mission_base.h"

float Mission_base::limit_av(float av,float limit)
{
    return (abs(av) > limit) ? limit * abs(av) / av : av;
}

void Mission_base::set_posvel(int32_t lat, int32_t lon, float alt, float _vx, float _vy, float _vz, float _yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore) //yaw_ignore==1 do not consider yaw
{
    // Vector3f pos_ned;
    // // sanity check location
    // if (!check_latlng(lat, lon))
    //     return;
    // Location loc;
    // loc.lat = lat;
    // loc.lng = lon;
    // loc.alt = alt * 100;
    // loc.flags.relative_alt = true;
    // loc.flags.terrain_alt = false;
    // pos_ned = plane.pv_location_to_vector(loc);
    // // prepare yaw
    // float yaw_cd = 0.0f;
    // bool yaw_relative = false;
    // float yaw_rate_cds = 0.0f;
    // if (!yaw_ignore)
    // {
    //     yaw_cd = _yaw * 100.0f;
    // }
    // if (!yaw_rate_ignore)
    // {
    //     yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    // }
    // float limit=plane.wp_nav->get_speed_xy();
    // _vx=limit_av(_vx,limit);
    // _vy=limit_av(_vy,limit);
    // _vz=limit_av(_vz,limit);
    // plane.guided_set_destination_posvel(pos_ned, Vector3f(_vx * 100.0f, _vy * 100.0f, -_vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}

void Mission_base::set_attitude()
{
}

void Mission_base::upload_pvy()
{
    Pxyz.lat = plane.current_loc.lat;
    Pxyz.lng = plane.current_loc.lng;
    Pxyz.alt = plane.current_loc.alt; //cm
    const Vector3f &vel = plane.gps.velocity();
    vx = vel.x/100.0f; //  m/s
    vy = vel.y/100.0f;
    vz = vel.z/100.0f;//up
    yaw = plane.ahrs.yaw_sensor/100.0f;
}

void Mavlist::update(mavlink_global_position_int_t packet)
{
    time_boot_ms=packet.time_boot_ms;
    Pxyz.lat = packet.lat;
    Pxyz.lng = packet.lon;
    altitude = packet.alt;   //mm   no use
    Pxyz.alt =(int32_t)packet.relative_alt/10.0+500;  //cm
    vx=packet.vx/100.0f;   //m/s
    vy=packet.vy/100.0f;
    vz=packet.vz/100.0f; //up
    yaw=packet.hdg/100.0f; 
}

#endif
