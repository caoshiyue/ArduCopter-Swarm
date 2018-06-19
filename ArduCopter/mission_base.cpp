#if MISSION_MANAGE == ENABLED
#include "Copter.h"
#include "mission_base.h"

<<<<<<< HEAD
float Mission_base::limit_av(float av,float limit)
{
    if(fabsf(av)>limit)
        return av/fabsf(av)*limit;
    else
        return av;
}

void Mission_base::set_postion(int32_t lat, int32_t lon, float alt, float _yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore) //yaw_ignore==true do not consider yaw
=======
void Mission_base::set_postion(int32_t lat, int32_t lon, int32_t alt, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false) //yaw_ignore==true do not consider yaw
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
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
<<<<<<< HEAD
        yaw_cd = ToDeg(_yaw) * 100.0f;
=======
        yaw_cd = ToDeg(yaw) * 100.0f;
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
    copter.guided_set_destination(pos_ned, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}
<<<<<<< HEAD
void Mission_base::set_velocity(float _vx, float _vy, float _vz, float _yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore)
=======
void Mission_base::set_velocity(float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false)
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
{
    // prepare yaw
    float yaw_cd = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_cds = 0.0f;
    if (!yaw_ignore)
    {
<<<<<<< HEAD
        yaw_cd = ToDeg(_yaw) * 100.0f;
=======
        yaw_cd = ToDeg(yaw) * 100.0f;
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
<<<<<<< HEAD
    float limit=copter.wp_nav->get_speed_up();
    _vx=limit_av(_vx,limit);
    _vy=limit_av(_vy,limit);
    _vz=limit_av(_vz,limit);
    status_int[0]=(uint8_t)_vx*10;
    status_int[1]=(uint8_t)_vy*10;
    status_int[2]=(uint8_t)_vz*10;
    copter.guided_set_velocity(Vector3f(_vx * 100.0f, _vy * 100.0f, -_vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}

void Mission_base::set_posvel(int32_t lat, int32_t lon, float alt, float _vx, float _vy, float _vz, float _yaw, bool yaw_rate_ignore, float yaw_rate, bool yaw_ignore) //yaw_ignore==1 do not consider yaw
=======
    copter.guided_set_velocity(Vector3f(vx * 100.0f, vy * 100.0f, -vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}

void Mission_base::set_posvel(int32_t lat, int32_t lon, int32_t alt, float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25, bool yaw_ignore = false) //yaw_ignore==1 do not consider yaw
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
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
<<<<<<< HEAD
        yaw_cd = ToDeg(_yaw) * 100.0f;
=======
        yaw_cd = ToDeg(yaw) * 100.0f;
>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(yaw_rate) * 100.0f;
    }
<<<<<<< HEAD
    float limit=copter.wp_nav->get_speed_up();
    _vx=limit_av(_vx,limit);
    _vy=limit_av(_vy,limit);
    _vz=limit_av(_vz,limit);
    copter.guided_set_destination_posvel(pos_ned, Vector3f(_vx * 100.0f, _vy * 100.0f, -_vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
}
void Mission_base::upload_pvy()
{
    Pxyz.lat = copter.current_loc.lat;
    Pxyz.lng = copter.current_loc.lng;
    Pxyz.alt = copter.current_loc.alt; //cm
    const Vector3f &vel = copter.inertial_nav.get_velocity();
    vx = vel.x/100.0f; //  m/s
    vy = vel.y/100.0f;
    vz = vel.z/100.0f;//up
    yaw = copter.ahrs.yaw_sensor/100.0f;
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
=======
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

>>>>>>> 9197220d5a0bff7b258ec924d41f3a727a539b45
}

#endif
