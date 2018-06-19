#if MISSION_MANAGE==ENABLED
#include "Copter.h"
#include "mission_base.h"
#define mission_number 1

static uint8_t mission_status=0;
static uint8_t mission_id=0;
static uint8_t counter=0;
Mission_base *mission_list[mission_number]={new Mission_pigeon()};                   // add missions need to modify

// mission loop - 10hz
void Copter::mission_manage() 
{
    if(copter.control_mode != GUIDED && copter.control_mode != LOITER)
        return ;
    if(mission_status==2)//restart or init 2
    {
        mission_list[mission_id]->init();
        if(mission_list[mission_id]->init_flag)
            mission_status=1;
    }
    else if(mission_status==1 && mission_list[mission_id]->init_flag)//iteration 1
    {
	if(copter.control_mode == GUIDED)
        	mission_list[mission_id]->run();
        upload_posvel(mavlink_channel_t(5));    //MAVLINK_COMM_5 broatcast
    }
    // suspend 0
    if(counter%16==0)
        send_mission_status(MAVLINK_COMM_2);
    counter++;
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
    mission_list[id]->set_para(p_10,p_15);
}
void NOINLINE Copter::send_mission_status(mavlink_channel_t chan)// maybe can use it as a heartbeat between flock, scan nods and get id
{
    mavlink_msg_mission_status_send(chan, mission_id, mission_status, mission_list[mission_id]->status_int, mission_list[mission_id]->status_float);//debug variable in missionlist[]
}
void Copter::upload_posvel(mavlink_channel_t chan)
{
    //CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
    copter.send_location(chan);
}
void Copter::record_flock_posvel(uint8_t mav_id,mavlink_global_position_int_t packet)
{
    if(mav_id<MAX_FLOCK_NUM)
      mission_list[mission_id]->mav[mav_id].update(packet);
}
#endif
