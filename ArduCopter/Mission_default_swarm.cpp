#if MISSION_MANAGE == ENABLED
#include "mission_base.h"

void Mission_default_swarm::set_para(uint8_t* p_10, float* p_15)
{
    up=*p_10;
     for(int i=0;i<=2;i++)  
     {
        disx[i+1] = *(p_15+i); 
        disy[i+1] = *(p_15+3+i); 
     }
     vm=*(p_15+6);
}
void Mission_default_swarm::init()
{
    if(this_id==1||fabsf(disx[1])>0.1||fabsf(disy[1])>0.1)
        init_flag=true;
}

void Mission_default_swarm::run()
{
    Vector2f dp=location_diff(mav[up].Pxyz,Pxyz);   //real distance /m
    float distancex = disx[1] * cos(mav[up].yaw*3.1415926/180.0f) - disy[1] * sin(mav[up].yaw*3.1415926/180.0f); // target distance /m
    float distancey = disx[1] * sin(mav[up].yaw*3.1415926/180.0f) + disy[1] * cos(mav[up].yaw*3.1415926/180.0f); 
    Location target_location=mav[up].Pxyz;      //target location /Wsg84
    location_offset(target_location, distancex + mav[up].vx*0.1, distancey + mav[up].vy*0.1);
    float target_vx=-(dp.x-distancex)*proportion_v;
    float target_vy=-(dp.y-distancey)*proportion_v;
    float target_vz=(Pxyz.alt - mav[up].Pxyz.alt)/100.0f*proportion_v; //z down
    if(this_id!=1)
        set_posvel(target_location.lat,target_location.lng,target_location.alt,
        limit_av(target_vx,vm),limit_av(target_vy,vm),limit_av(target_vz,vm),mav[up].yaw);
}
//-------------------------------------- Private --------------------------------------


#endif
