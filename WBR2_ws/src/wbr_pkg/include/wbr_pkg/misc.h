#ifndef __MISC_H_
#define __MISC_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "math.h"

#define Robot_States_Nums 4

class Robot_Class{
    public:
        //机器人目标控制量
        //机器人关节控制量
        float target_hip_pos_k;  //运动学角度
        float target_knee_pos_k;    
        float target_hip_pos_m;  //执行电机角度
        float target_knee_pos_m; // 
        
        float target_left_wheel_trq;
        float target_right_wheel_trq;

        //机器人姿态
        float yaw; //rad
        float pitch; //rad
        float roll; //rad

        float RobotStates[Robot_States_Nums]; //机器人动态方程中的状态向量

        //机器人状态
        bool is_robot_init = 0;

        void InitRobot(); // 初始化机器人函数。主要是控制参数等。
        void DisableRobot(){ is_robot_init = 0;}
        void Inverse_kinematic_solver();

};


void Robot_Class::InitRobot()
{
    //机器人关节控制量
    target_hip_pos_k = 0;  //运动学角度
    target_knee_pos_k=0;    
    target_hip_pos_m=0;  //执行电机角度
    target_knee_pos_m=0; // 

    target_left_wheel_trq=0;
    target_right_wheel_trq=0;

    //机器人姿态
    yaw=0; //rad
    pitch=0; //rad
    roll=0; //rad

    //机器人状态
    is_robot_init = 1;
}

void Robot_Class::Inverse_kinematic_solver()
{

}


#endif