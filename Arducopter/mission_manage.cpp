#include "Copter.h"

#if MISSION_MANAGE==ENABLED
#include "publisher.h"
#include "mission_base.h"
using namespace CUSTOM_MISSION;
#define mission_number 2
/// static variable init
static uint8_t mission_status=0;
static uint8_t mission_id=0;

/// Mission list
//在将自定义任务添加到任务列表中，同时修改 mission_number
Mission_base *mission_list[mission_number]={new Mission_swarm_avoid()}; 

// mission loop - 20hz
void Copter::mission_manage() 
{
    if(!copter.flightmode->in_guided_mode() && copter.control_mode != LOITER)
        return ;
    if(mission_status==2)//init 2
    {
        mission_list[mission_id]->init();
        if(mission_list[mission_id]->init_flag)
            mission_status=1;
    }
    else if(mission_status==1 && mission_list[mission_id]->init_flag)//run 1
    {   
        Mission_base::pub.notify();
        Mission_base::update_mypvy();
        mission_list[mission_id]->send_regist();
        mission_list[mission_id]->check_position();
        if(copter.flightmode->in_guided_mode())
        	mission_list[mission_id]->run();
    }
    // suspend 0
    Mission_base::counter++;    // counter only can be modified here
}

void Copter::handle_mission_select(uint8_t start,uint8_t id)
{
    if(id>mission_number-1)
        return ;
    mission_status=start;
    mission_id=id;            // id = 0 ~ x
} 

void Copter::handle_set_para(uint8_t id, uint8_t* p_10,float* p_15)
{
    //TODO: 增加订阅数量
    mission_list[id]->regist_leader=*p_10;
    mission_list[id]->regist_leader+= (*(p_10+1))<<8; 
    mission_list[mission_id]->send_regist();
    mission_list[id]->set_para(p_10,p_15);
    mission_list[mission_id]->send_mission_status(gcs_chan, 1, "parameter received");
}

void Copter::handle_register(uint8_t mav_id,uint8_t regist)
{
    Mission_base::pub.handle_regist_msg(mav_id,regist);
}

void Copter::handle_flock_posvel(uint8_t mav_id,mavlink_global_position_int_t packet)
{
     if(mav_id<MAX_FLOCK_NUM)
        Mission_base::sub.handle_notify_msg(mav_id,packet);
}
void NOINLINE Mission_base::send_mission_status(mavlink_channel_t chan, uint8_t interval, const char *fmt, ...)
{
    if(counter%interval!=0)
        return ;
    va_list arg_list;
    va_start(arg_list, fmt);
    char text[51];
    hal.util->vsnprintf(text, sizeof(text), fmt, arg_list);
    mavlink_msg_mission_status_send(chan, mission_id, mission_status, text);
    va_end(arg_list);
}

#endif
