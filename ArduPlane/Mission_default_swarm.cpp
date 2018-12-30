#if MISSION_MANAGE == ENABLED
#include "mission_base.h"

void Mission_default_swarm::set_para(uint8_t* p_10, float* p_15)
{
    up = *p_10;
    disx = *(p_15);
    disy = *(p_15 + 1);
    vm = *(p_15 + 2);
}
void Mission_default_swarm::init()
{
    if(this_id==1||fabsf(disx)>0.1||fabsf(disy)>0.1)
        init_flag=true;
    else
        init_flag=false;
}

void Mission_default_swarm::run()
{
    // Vector2f dp=location_diff(mav[up].Pxyz,Pxyz);   //real distance /m
    // float distancex = disx * cos(mav[up].yaw*3.1415926/180.0f) - disy * sin(mav[up].yaw*3.1415926/180.0f); // target distance /m
    // float distancey = disx * sin(mav[up].yaw*3.1415926/180.0f) + disy * cos(mav[up].yaw*3.1415926/180.0f); 
    // Location target_location=mav[up].Pxyz;      //fetch leader position     target location /Wsg84 
    // location_offset(target_location, distancex + mav[up].vx*0.1, distancey + mav[up].vy*0.1);//my target location
    // float target_vx=-(dp.x-distancex)*proportion_v;// proportional control
    // float target_vy=-(dp.y-distancey)*proportion_v;
    // float target_vz=(Pxyz.alt - mav[up].Pxyz.alt)/100.0f*proportion_v; //z down
    // if(this_id!=1)
    //     set_posvel(target_location.lat,target_location.lng,target_location.alt,
    //     limit_av(target_vx,vm),limit_av(target_vy,vm),limit_av(target_vz,vm),mav[up].yaw);
}
//-------------------------------------- Private --------------------------------------


#endif
