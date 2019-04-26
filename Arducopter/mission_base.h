/*
 * @Author: Csy
 * @LastEditors: Csy
 * @Description: 
 * @Date: 2019-03-06 16:03:19
 * @LastEditTime: 2019-04-26 20:51:40
 */

#pragma once
#include "Copter.h"

#if MISSION_MANAGE == ENABLED
#include "publisher.h"

namespace CUSTOM_MISSION
{
#ifndef this_id
#define this_id mavlink_system.sysid // sysid of this copter
#endif 


class Mission_base
{
public:
  Mission_base() {}
  ~Mission_base() {}

//-------------------------允许访问的属性和方法-----------------------------
  static Location my_Pxyz; // 自己位置，高度单位厘米  int32 alt in cm positive up 
  static float my_vx ;//自身速度 m/s
  static float my_vy ;
  static float my_vz ;
  static float my_yaw;
  bool init_flag = false; //初始化标志位,在初始化函数中，任务初始化完成后需要置true
  /**
   * @description: 限幅函数
   * @param：a 输入, limit 幅值
   */
  static float limit_av(float a, float limit);  
  /**
   * @description: 发送调试信息
   * @param {type} 
   * chan 指定飞机或地面站
   * interval 发送间隔，每interval次迭代执行一次,最大255
   * fmt 字符串信息，与printf用法相同
   * @return: void
   */ 
  void send_mission_status(mavlink_channel_t chan, uint8_t interval, const char *fmt, ...); 
  /**
   * @description: 设置位置，速度，位置和速度
   * @param {type} 同set_position_target_global_int
   * @return: void
   */
  void set_postion(int32_t lat, int32_t lon, float alt, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);
  void set_velocity(float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);
  void set_posvel(int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, float yaw = 0, bool yaw_rate_ignore = true, float yaw_rate = 0.25);

//-------------------------需要继承的方法-----------------------------
  /**
   * @description: 将收到的参数与任务参数对应
   * @param {type} 
   * p_10 10个整型参数
   * p_15 15个浮点参数
   * @return: void
   */
  virtual void set_para(uint8_t *p_10, float *p_15) = 0;
  /**
   * @description: 初始化方法, 任务第一次执行或重启时执行
   * @return: void
   */
  virtual void init() = 0;
  /**
   * @description: 任务主循环，在Guided模式下任务启动后将被循环调用，20Hz
   * @return: void
   */
  virtual void run() = 0;
//-------------------------不建议访问的属性和方法-----------------------------

  static uint8_t counter; // 自加迭代器，用于时时序同步
  static publisher pub;   // 发布器，收订阅请求，发自己位置
  static subscriber sub;  // 订阅器，发订阅请求，收其他飞机位置
  uint16_t regist_leader=0; //需要订阅的飞机id，bitmask

  /**
   * @description: 从AHRS更新自身状态
   * @return: void
   */
  static void update_mypvy(); 
  /**
   * @description: 获取指定id的飞机状态，用法  get_mav(id).Pxyz.alt  get_mav(id).yaw
   * @param: 飞机id
   * @return: Mav_status
   */
  static Mav_status get_mav(uint8_t); 
  /**
   * @description: 发送订阅信息
   * @param {type} 
   * @return: void
   */
  void send_regist();
  /**
   * @description: 将设置好的位置速度传递给消息处理方法
   * @param {type} mavlink_set_position_target_global_int_t 消息包
   * @return: void
   */
  void set_position_target_global(mavlink_set_position_target_global_int_t &packet);
  /**
   * @description: 检查其他飞机的位置时效
   * @param {type} void
   * @return: void
   */
  void check_position(void);

};

/*
/**
 * @description: 常用函数，可直接调用，计算距离已知坐标点一定距离的点的坐标位置
 * @param {type} 
 * ofs_north  x距离，北为正，单位米
 * ofs_east   y距离，东为正，单位米
 * loc        已知坐标点，调用后计算好的坐标将保存在此
 * @return: void
 * void  location_offset(struct Location &loc, float ofs_north, float ofs_east);
 */

/**
 * @description: 常用函数，可直接调用，计算坐标2相对于坐标1的矢量距离,loc2-loc1,单位米
 * @param {type} 
 * loc1 坐标1
 * loc2 坐标2
 * @return: 距离矢量，北x正，东y正
 * Vector2f    location_diff(const struct Location &loc1, const struct Location &loc2);    
 */

/*
注意:
sin() atan()... in rad
*/

//自定义任务示例，主从跟随任务
//需要在 mission_manage 中添加进任务列表
//-----------------------------------Mission zero ----------------------------------------------
class Mission_default_swarm : public Mission_base
{
public:
  Mission_default_swarm() {}
  ~Mission_default_swarm() {}
  /**
   * @description: 重写基类虚函数
   */
  void set_para(uint8_t *, float *);
  void init();
  void run();

private:                    //自定义变量，不建议在主循环中定义变量
  float disx = 0;           // offset of my position if leader is origin in NED coordinate  
  float disy = 0;           //      
  float vm = 3.0;           //max velocity           
  float proportion_v = 0.2; //proportion control gain of velocity    
  uint8_t up;               //  leader id     
};

//-----------------------------------Mission one----------------------------------------------
class Mission_pigeon : public Mission_base
{
public:
  Mission_pigeon() {}
  ~Mission_pigeon() {}
  void set_para(uint8_t *, float *);
  void init();
  void run();

private: //parameters and algorithm
  float dis[4] = {0};
  float disx[4] = {0}; // NED coordinate     //      P
  float disy[4] = {0}; //      P
  uint8_t upper[4] = {0};

  float K = 0.01;
  uint8_t kp = 4; //      P
  uint8_t kv = 2; //      P
  float w1 = 2.0; //      P
  uint8_t kh = 3; //      P
  float w2 = 2.0; //      P
  uint8_t m = 1;
  float tv = 0.1;
  float vm = 2.0;
  float am = 3.0;
  float k2x1 = 0.4; //      P
  float k2x2 = 0.4; //      P
  uint8_t kxy = 150;

  float force_target(bool x);
  float force_form(uint8_t i, bool x);
  float repulsion(float dis, float range, float sudden, bool x);
  float u_x();
  float u_y();
  float u_z();
  void upload_var();
  void get_upper();
};

//-----------------------------------Mission two----------------------------------------------
class Mission_swarm_avoid : public Mission_base
{
public:
  Mission_swarm_avoid() {}
  ~Mission_swarm_avoid() {}
  void set_para(uint8_t *, float *);
  void init();
  void run();

private:                    //parameters and algorithm
  float disx = 0;           // offset of my position if leader is origin in NED coordinate  P
  float disy = 0;           //      P
  float vm = 1.0;           //max velocity           P
  float proportion_v = 0.2; //proportion control gain of velocity    P
  uint8_t up;               //  leader id     P
  uint8_t obstacle[3];

  float distancex = 0;
  float distancey = 0;
  float target_vx = 0;
  float target_vy = 0;
  float target_vz = 0;
  float distance_to_leadery=0;
  float abs_distance_to_obstacle=0;
  float detect_distance=0;
};

} //namespace
#endif
