#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "mission_base.h"
using namespace CUSTOM_MISSION;

void Mission_swarm_avoid::set_para(uint8_t* p_10, float* p_15)
{
    up = *(p_10+2);
    obstacle[0]=*(p_10+3);
    obstacle[1]=*(p_10+4);
    obstacle[2]=*(p_10+5);
    disx = *(p_15);
    disy = *(p_15 + 1);
    vm = *(p_15 + 2);
    detect_distance  = *(p_15 + 3);
}
void Mission_swarm_avoid::init()
{
    if(this_id==1||fabsf(disx)>0.1||fabsf(disy)>0.1)
    {
        init_flag=true;
    }
    else
        init_flag=false;
}

void Mission_swarm_avoid::run()
{
    send_mission_status(gcs_chan, 64, "yaw %.2f from leader %d", get_mav(up).yaw,up);
    if(my_Pxyz.alt<100)
        return;
    Vector2f dp = location_diff(get_mav(up).Pxyz, my_Pxyz);    //real distance /m
    if(abs(dp.x)>1000||abs(dp.y)>1000)      
        return;

    Vector2f distance_to_obstacle;
    float distance_to_obstacle_y=0;
    distance_to_leadery = disy;
    for (uint8_t i = 0; i < 3; i++)
    {
        distance_to_obstacle = location_diff(get_mav(obstacle[i]).Pxyz, my_Pxyz);
        if(abs(distance_to_obstacle.x)>1000||abs(distance_to_obstacle.y)>1000)
            continue;
        abs_distance_to_obstacle = powf(powf(distance_to_obstacle.x, 2) + powf(distance_to_obstacle.y, 2), 0.5);
        if (abs_distance_to_obstacle < detect_distance)
        {
            distance_to_obstacle_y = -distance_to_obstacle.y * cosf(my_yaw* M_PI / 180.0f) + distance_to_obstacle.x * sinf(my_yaw* M_PI / 180.0f);
            distance_to_leadery = (distance_to_obstacle_y > 0) ? (disy - 1.5) : (disy + 1.5);
            break;
        }
        else
            distance_to_leadery = disy;
    }
    distancex = disx * cosf(get_mav(up).yaw * M_PI / 180.0f) - distance_to_leadery * sinf(get_mav(up).yaw * M_PI / 180.0f); // target distance /m
    distancey = disx * sinf(get_mav(up).yaw * M_PI / 180.0f) + distance_to_leadery * cosf(get_mav(up).yaw * M_PI / 180.0f);
    Location target_location = get_mav(up).Pxyz;                                                            //fetch leader position     target location /Wsg84
    location_offset(target_location, distancex + get_mav(up).vx * 0.05, distancey + get_mav(up).vy * 0.05); //my target location
    target_vx = -(dp.x - distancex) * proportion_v;                                                   // proportional control
    target_vy = -(dp.y - distancey) * proportion_v;
    target_vz = (get_mav(up).Pxyz.alt-my_Pxyz.alt) / 100.0f * proportion_v; //z up
    if (this_id != 1)
        set_posvel(target_location.lat, target_location.lng, target_location.alt/100.0,
                   limit_av(target_vx, vm), limit_av(target_vy, vm), limit_av(target_vz, vm), get_mav(up).yaw);
}
//-------------------------------------- Private --------------------------------------


#endif
