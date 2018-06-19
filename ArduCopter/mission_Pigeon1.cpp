#if MISSION_MANAGE == ENABLED
#include "mission_base.h"

void Mission_pigeon::set_para(uint8_t* p_10, float* p_15)
{
     kp = *(p_10); 
     kv = *(p_10+1);  
     kh = *(p_10+2);
     for(int i=0;i<=2;i++)  
     {
        disx[i+1] = *(p_15+i); 
        disy[i+1] = *(p_15+3+i); 
     }
     w1 = *(p_15+6);  
     w2 = *(p_15+7);  
     k2x1 = *(p_15+8); 
     k2x2 = *(p_15+9); 
}
void Mission_pigeon::init()
{
    get_upper();
    if(this_id==1||fabsf(disx[1])>0.1||fabsf(disy[1])>0.1)
        init_flag=true;
}

void Mission_pigeon::run()
{
    upload_pvy();
    upload_var();
    if(this_id!=1)
        set_velocity(tv*limit_av(u_x(),am)+vx,tv*limit_av(u_y(),am)+vy,-limit_av(u_z(),am)*tv+vz,mav[1].yaw);
}

//------------------------------private-----------------------------------
float Mission_pigeon::force_target(bool x) //x==true
{
    uint8_t up = upper[1];
    Vector2f dp=location_diff(mav[up].Pxyz,Pxyz);
    status_float[0]=dp.x;
    status_float[1]=dp.y;	
    status_float[2]=mav[1].Pxyz.alt;
    if (up == 0)
        return 0;
    if (x)
    {
        float distance = disx[1] * cos(yaw*3.1415926/180.0f) - disy[1] * sin(yaw*3.1415926/180.0f); //x
        return (pow(1.2, dp.x - distance) - pow(1.2, -(dp.x - distance))) * k2x1 + atan(dp.x - distance) / 3.1415 * 3 - repulsion(1, 1.5, 1, true);
    }
    else
    {
        float distance = -disx[1] * sin(yaw*3.1415926/180.0f) + disy[1] * cos(yaw*3.1415926/180.0f); //y
        return (pow(1.2, dp.y - distance) - pow(1.2, -(dp.y - distance))) * k2x1 + atan(dp.y - distance) / 3.1415 * 3 - repulsion(1, 1.5, 1, false);
    }
}

float Mission_pigeon::force_form(uint8_t i, bool x)
{
    uint8_t up = upper[i];
    Vector2f dp=location_diff(mav[up].Pxyz,Pxyz);
    if (up != 0)
    {
        float X = fabsf(dp.x);
        float Y = fabsf(dp.y);
        float pd = pow(pow(X, 2) + pow(Y, 2), 0.5);
        float XY = MAX(pow(X, 2) + pow(Y, 2), 1.01);
        if (x)
        {
            int8_t sign = (dp.x) / X;
            return (kxy * (X / XY - pow(dis[i], 2) * X / pow(XY, 2)) + k2x2 * (pow(1.2, pd - dis[i]) - 1) * X / pd) * sign;
        }
        else
        {
            int8_t sign = (dp.y) / Y;
            return (kxy * (Y / XY - pow(dis[i], 2) * Y / pow(XY, 2)) + k2x2 * (pow(1.2, pd - dis[i]) - 1) * Y / pd) * sign;
        }
    }
    else
        return 0;
}
float Mission_pigeon::repulsion(float dist,float range, float sudden,bool x)
{
    float repulsionx = 0;
    float repulsiony = 0;
    for (int j = 1; j <= 3; j += 1)
    {
        uint8_t up = upper[j];
        if (up != 0)
        {
            Vector2f dp=location_diff(mav[up].Pxyz,Pxyz);
            float XY = MAX(pow(dp.x, 2) + pow(dp.y, 2), 1.01);
            if (XY < range)
            {
                repulsionx = repulsionx + pow(dist, 2) * dp.x / pow((XY - sudden), 2);
                repulsiony = repulsiony + pow(dist, 2) * dp.y / pow((XY - sudden), 2);
            }
        }
    }
    if (x)
        return repulsionx;
    else
        return repulsiony;
}
float Mission_pigeon::u_x()
{
    float u = 0;

    if (this_id == 2 || this_id == 3)
    {
        if (upper[1] != 0)
        {
            float dv = vx - mav[1].vx;
            u = (-kp * force_target(true) - kv * dv) * w1;
        }
    }
    else
    {
        for (int i = 1; i <= 3; i += 1)
        {
            uint8_t up = upper[i];
            if (up != 0)
            {
                float dv = vx - mav[up].vx;
                u = u + (-kp * force_form(i, true) - kv * dv) * w1;
            }
        }
    }
    if (upper[1] != 0)
        u = u - m * (vx - mav[1].vx) * w1 + K * vx;
    return u;//limit_av(u,am);
    
}

float Mission_pigeon::u_y()
{
    float u = 0;
    if (this_id == 2 || this_id == 3)
    {
        if (upper[1] != 0)
        {
            float dv = vy - mav[upper[1]].vy;
            u = (-kp * force_target(false) - kv * dv) * w1;
        }
    }
    else
    {
        for (int i = 1; i <= 3; i += 1)
        {
            uint8_t up = upper[i];
            if (up != 0)
            {
                float dv = vy - mav[up].vy;
                u = u + (-kp * force_form(i, false) - kv * dv) * w1;
            }
        }
    }
    if (upper[1] != 0)
        u = u - m * (vy - mav[1].vy) * w1 + K * vy;
    return limit_av(u,am);
}
float Mission_pigeon::u_z()
{
    float u = 0;
    for (int i = 1; i <= 3; i += 1)
    {
        uint8_t up = upper[i];
        if (up != 0)
        {
            float dp = (Pxyz.alt - mav[up].Pxyz.alt)/100.0f;
            float dv = vz - mav[up].vz;
            u = u + (-kh * dp - kv * dv) * w2;
        }
    }
    if (upper[1] != 0)
        u = u - m * mav[1].vz * w2 + K * vz;
    return u;
}
void Mission_pigeon::upload_var()
{
    for (int i = 0; i < 4; i++)
        dis[i] = pow(pow(disx[i], 2) + pow(disy[i], 2), 0.5);
}

void Mission_pigeon::get_upper()
{
    uint8_t id=this_id;
    if (id == 2)
        upper[1] = 1;
    if (id == 3)
    {
        upper[1] = 1;
        upper[2] = 2;
    }
    if (id == 4)
    {
        upper[1] = 1;
        upper[2] = 2;
        upper[3] = 3;
    }
    if (id > 4 && id%2 == 0)
    {
        upper[1] = id - 4;
        upper[2] = id - 2;
        upper[3] = id - 1;
    }
    if (id > 4 && id%2 == 1)
    {
        upper[1] = id - 4;
        upper[2] = id - 3;
        upper[3] = id - 2;
    }
}


#endif
