#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "publisher.h"

namespace CUSTOM_MISSION
{
/**
 * TODO: 订阅数量增加，广播
 */
void publisher::notify()
{
	if (mav_sub.size() > 6)
		copter.send_location(broad_chan);
	else
		for (auto iter = mav_sub.begin(); iter != mav_sub.end(); iter++)
			copter.send_location(get_mav_chan(*iter));
}

void publisher::handle_regist_msg(uint8_t remote_id, bool a)
{
	if(remote_id==this_id)
		return;
	auto iter = mav_sub.begin();
	for ( ;iter != mav_sub.end(); iter++)
	{
		if (*iter == remote_id)
			break;
	}

	if (a)
	{
		if (iter == mav_sub.end())
			mav_sub.push_back(remote_id);
		else
			return;
	}
	else
	{
		if (iter == mav_sub.end())
			return;
		else
			mav_sub.erase(iter);
	}
}
void subscriber::regist(uint8_t target_id, bool a)
{
	if(target_id==this_id)
		return;
	mavlink_msg_sub_pub_register_send(get_mav_chan(target_id), (uint8_t)a);
}

void subscriber::handle_notify_msg(uint8_t remote_id,mavlink_global_position_int_t packet)
{
	mav_map[remote_id].update(packet);
}

mavlink_channel_t get_mav_chan(uint8_t id)
{
    if(id>0&&id<=MAX_FLOCK_NUM)
        return mavlink_channel_t(id+5);
    else
        return mavlink_channel_t(5);
}
  
}
#endif