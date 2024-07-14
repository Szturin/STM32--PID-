#include "stm32f10x.h"                  // Device header
#include "PID.h"


float laser_Pose_control_Y(float Expet ,float Measure)
{
    static float Kp = 0.005;
    static float Ki = 0.0035;
    static float Kd = 0.008;
    //误差，上一次误差，上上一次误差
    static float ERR,Last_ERR,LastLast_ERR;
    float Out_Inc;
    //误差更新
    ERR = Expet - Measure;
    //计算

    Out_Inc = Kp*(ERR - Last_ERR ) + Ki*Last_ERR + Kd*(ERR - 2*Last_ERR + LastLast_ERR);

    //误差更新

    Last_ERR = ERR;

    LastLast_ERR = Last_ERR;

    return Out_Inc;
}

float laser_Pose_control_X(float Expet ,float Measure)
{
    static float Kp = 0.003;
    static float Ki = 0.005;
    static float Kd = 0.000;
    //误差，上一次误差，上上一次误差
    static float ERR,Last_ERR,LastLast_ERR;
    float Out_Inc;
    //误差更新
    ERR = Expet - Measure;
    //计算

    Out_Inc = Kp*(ERR - Last_ERR ) + Ki*Last_ERR + Kd*(ERR - 2*Last_ERR + LastLast_ERR);

    //误差更新

    Last_ERR = ERR;

    LastLast_ERR = Last_ERR;

    return Kp*(ERR - Last_ERR ) + Ki*Last_ERR + Kd*(ERR - 2*Last_ERR + LastLast_ERR);
}