#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "mission_base.h"
using namespace CUSTOM_MISSION;

void Mission_base::set_postion(int32_t lat, int32_t lon, float alt, float yaw, bool yaw_rate_ignore, float yaw_rate) 
{
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = millis();
    packet.lat_int = lat;
    packet.lon_int = lon;
    packet.alt = alt;
    packet.vx = 0;
    packet.vy = 0;
    packet.vz = 0;
    packet.afx = 0;
    packet.afy = 0;
    packet.afz = 0;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = ~MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    set_position_target_global(packet);
}
void Mission_base::set_velocity(float vx, float vy, float vz, float yaw, bool yaw_rate_ignore, float yaw_rate)
{
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = millis();
    packet.lat_int = 0;
    packet.lon_int = 0;
    packet.alt = 0;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = -vz;
    packet.afx = 0;
    packet.afy = 0;
    packet.afz = 0;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = ~MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    set_position_target_global(packet);
}

void Mission_base::set_posvel(int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, float yaw, bool yaw_rate_ignore, float yaw_rate) 
{
    mavlink_set_position_target_global_int_t packet;
    packet.time_boot_ms = millis();
    packet.lat_int = lat;
    packet.lon_int = lon;
    packet.alt = alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = -vz;
    packet.afx = 0;
    packet.afy = 0;
    packet.afz = 0;
    packet.yaw = yaw;
    packet.yaw_rate = yaw_rate;
    packet.type_mask = ~MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    packet.target_system = 0;
    packet.target_component = 0;
    packet.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    set_position_target_global(packet);
}

void Mission_base::set_position_target_global(mavlink_set_position_target_global_int_t &packet)
{
    bool pos_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
    bool vel_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
    bool acc_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
    bool yaw_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
    bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
    bool alt_ignore = packet.type_mask & (1<<2);

    /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         */

    Vector3f pos_neu_cm; // position (North, East, Up coordinates) in centimeters

    if (!pos_ignore)
    {
        // sanity check location
        if (!check_latlng(packet.lat_int, packet.lon_int))
        {
            return;
        }
        Location::ALT_FRAME frame;
        /// This function cannot be invoked, so rewrite it 
        // if (!mavlink_coordinate_frame_to_location_alt_frame(packet.coordinate_frame, frame)) 
        // {
        //     // unknown coordinate frame
        //     return;
        // }

        if (packet.coordinate_frame == MAV_FRAME_GLOBAL_RELATIVE_ALT || packet.coordinate_frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
            frame = Location::ALT_FRAME_ABOVE_HOME;
        else
            return;

        const Location loc{
            packet.lat_int,
            packet.lon_int,
            int32_t(packet.alt * 100),
            frame,
        };
        if (!loc.get_vector_from_origin_NEU(pos_neu_cm))
        {
            return;
        }
    }

    // prepare yaw
    float yaw_cd = 0.0f;
    bool yaw_relative = false;
    float yaw_rate_cds = 0.0f;
    if (!yaw_ignore)
    {
        yaw_cd = ToDeg(packet.yaw) * 100.0f;
        yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
    }
    if (!yaw_rate_ignore)
    {
        yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
    }

    if (!pos_ignore && !vel_ignore && acc_ignore)
    {
        copter.mode_guided.set_destination_posvel(pos_neu_cm, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    }
    else if (pos_ignore && !vel_ignore && acc_ignore)
    {
        if(!alt_ignore) // hold alt 
            packet.vz = -(pos_neu_cm.z-my_Pxyz.alt)*0.005; // positive down
        copter.mode_guided.set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f), !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    }
    else if (!pos_ignore && vel_ignore && acc_ignore)
    {
        copter.mode_guided.set_destination(pos_neu_cm, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
    }
    return;
}

void Mav_status::update(mavlink_global_position_int_t packet)
{
    time_boot_ms=packet.time_boot_ms;
    Pxyz.lat = packet.lat;
    Pxyz.lng = packet.lon;
    altitude = packet.alt;   //mm   no use
    Pxyz.alt =(int32_t)packet.relative_alt/10.0;  //mm . cm positive up
    vx=packet.vx/100.0f;   //m/s
    vy=packet.vy/100.0f;
    vz=packet.vz/100.0f; // positive up
    yaw=packet.hdg/100.0f; 
}

Location Mission_base::my_Pxyz=Location();
float Mission_base::my_vx =0; //my
float Mission_base::my_vy =0; 
float Mission_base::my_vz =0; //po
float Mission_base::my_yaw=0; 
uint8_t Mission_base::counter=0;
publisher Mission_base::pub=publisher();
subscriber Mission_base::sub=subscriber();

void Mission_base::update_mypvy()
{
    my_Pxyz.lat = copter.current_loc.lat;
    my_Pxyz.lng = copter.current_loc.lng;
    my_Pxyz.alt = copter.current_loc.alt; //cm
    const Vector3f &vel = copter.inertial_nav.get_velocity();
    my_vx = vel.x/100.0f; //  m/s
    my_vy = vel.y/100.0f;
    my_vz = vel.z/100.0f;//    positive up
    my_yaw = copter.ahrs.yaw_sensor/100.0f;
}
float Mission_base::limit_av(float av,float limit)
{
    return (abs(av) > limit) ? limit * abs(av) / av : av;
}

Mav_status Mission_base::get_mav(uint8_t a)
{
    return sub.mav_map[a];
}
#endif

/*
extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location &loc, float ofs_north, float ofs_east);

return the distance in meters as a N/E vector from loc1 to loc2     loc2-loc1 in Meter
Vector2f    location_diff(const struct Location &loc1, const struct Location &loc2);    
*/