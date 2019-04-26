#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "mission_base.h"
using namespace CUSTOM_MISSION;

void Mission_default_swarm::set_para(uint8_t* p_10, float* p_15)
{
    //通过地面站上传的自定义参数与算法变量匹配
    up = *(p_10+2);
    disx = *(p_15);
    disy = *(p_15 + 1);
    vm = *(p_15 + 2);
}
void Mission_default_swarm::init()
{
    if(this_id==1||fabsf(disx)>0.1||fabsf(disy)>0.1)   //参数正确上传，则初始化成功
    {
        init_flag=true;
    }
    else
        init_flag=false;
}

void Mission_default_swarm::run()
{
    send_mission_status(gcs_chan, 64, "yaw %.2f from leader %d", get_mav(up).yaw,up);//告知地面站收到长机信息
    if(my_Pxyz.alt<100) //高度小于1m不执行
        return;
    Vector2f dp = location_diff(get_mav(up).Pxyz, my_Pxyz);    //real distance /m
    if(abs(dp.x)>1000||abs(dp.y)>1000)   //距离长机1000m外不执行
        return;
    //依照长机航向确定队形，利用旋转矩阵计算目标点相对于长机的XY /m，
    float distancex = disx * cosf(get_mav(up).yaw * M_PI / 180.0f) - disy * sinf(get_mav(up).yaw * M_PI / 180.0f); 
    float distancey = disx * sinf(get_mav(up).yaw * M_PI / 180.0f) + disy * cosf(get_mav(up).yaw * M_PI / 180.0f);
    Location target_location = get_mav(up).Pxyz;                                                            //fetch leader position 
    location_offset(target_location, distancex + get_mav(up).vx * 0.05, distancey + get_mav(up).vy * 0.05); //计算目标点坐标
    float target_vx = -(dp.x - distancex) * proportion_v;                                                   // proportional control
    float target_vy = -(dp.y - distancey) * proportion_v;
    float target_vz = (get_mav(up).Pxyz.alt-my_Pxyz.alt) / 100.0f * proportion_v; //z up
    if (this_id != 1)// 1号机永远不执行，防止误切换模式
        set_posvel(target_location.lat, target_location.lng, target_location.alt/100.0,
                   limit_av(target_vx, vm), limit_av(target_vy, vm), limit_av(target_vz, vm), get_mav(up).yaw);
}
//-------------------------------------- Private --------------------------------------


#endif
