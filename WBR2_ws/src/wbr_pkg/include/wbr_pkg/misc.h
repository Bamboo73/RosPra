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

#define THIGH_LENGTH 0.2 //m
#define SHANK_LENGTH 0.18
#define WHEEL_RADIUS 0.08

#define MASS_BODY 6.9465 //kg
#define MASS_THIGH 2.258 // single thigh mass
#define MASS_SHANK 2.0872 // single thigh mass

#define MASS_WHEEL 1.6567

#define MAX_TARGET_H 0.38 // 根据高度求解逆运动学的时候所需要的上下高度界限。
#define MIN_TARGET_H 0.0845

class Robot_Class{
    public:
        //机器人参数
        float mass_body = MASS_BODY;
        float mass_both_shank = MASS_SHANK*2;
        float mass_both_thigh = MASS_THIGH*2;
        float mass_total = mass_body + mass_both_shank + mass_both_thigh; // 不需要计算轮子的位置。
        float l1 = THIGH_LENGTH;
        float l2 = SHANK_LENGTH;
        float r = WHEEL_RADIUS;


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
        void Inverse_kinematic_solver(float target_h);
        void Inverse_kinematic_solver2(float target_h);
};


void Robot_Class::InitRobot()
{
    //机器人参数
    float mass_body = MASS_BODY;
    float mass_both_shank = MASS_SHANK*2;
    float mass_both_thigh = MASS_THIGH*2;
    float mass_total = mass_body + mass_both_shank + mass_both_thigh; // 不需要计算轮子的位置。
    float l1 = THIGH_LENGTH;
    float l2 = SHANK_LENGTH;
    float r = WHEEL_RADIUS;

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

void Robot_Class::Inverse_kinematic_solver(float target_h)
{
    //进行高度的上下界判断
    double h;
    h = target_h > MAX_TARGET_H ? MAX_TARGET_H : target_h;
    h = h < MIN_TARGET_H ? MIN_TARGET_H : h;

    //进行是否进行了初始化的判断
    if(is_robot_init!=1)
        return; //没有初始化完毕，不允许求解逆运动学。
    //下述内容对逆运动学进行计算
    double A,B;
    A = l1 * (mass_total - 0.5*mass_both_thigh - mass_both_shank);
    B = l2 * (0.5 * mass_both_shank - mass_total); 
    // ROS_INFO("Solve A: %.3f B: %.3f ",A,B);

    double a,b,c; //一元二次函数的参数;
    a = (l1/l2)*(l1/l2) - (A/B)*(A/B);
    b = - 2*target_h*l1/ (l2*l2);
    c = target_h*target_h/(l2*l2) + (A/B)*(A/B) - 1;

    ROS_INFO("Solve a: %.3f b: %.3f c: %.3f",a,b,c);
    double cosa,cosb;
    cosa = (-b + sqrt(b*b - 4*a*c))/(2*a); 
    if(cosa <-1 || cosa > 1){ //判断结果是否在[-1 1]之间，根据几何意义，不会存在两个解都在此区间的情况
        cosa = (-b - sqrt(b*b - 4*a*c))/(2*a); 
    }
    //求解结果转换为关节参数进行下达
    target_hip_pos_k = (float)acos(cosa);
    double beta = asin( A * sin(target_hip_pos_k) / B );
    target_knee_pos_k = beta - target_hip_pos_k;
    double error =  h - (l1*cos(target_hip_pos_k) + l2*cos(beta));
    ROS_INFO("Solve cosa: %.3f  theta1: %.3f theta2: %.3f, error: %.3f",cosa,target_hip_pos_k,target_knee_pos_k,error);
  
    
    // 出于设计LQR控制器需要，得计算一下模拟倒立摆的杆长
    float lc;
    float y_c;
    y_c = (-mass_both_thigh*(0.5* l1 * cosa ) - (l1* cosa + 0.5*l2*cos(beta))*mass_both_shank)/mass_total;
    lc = h+y_c;

    ROS_INFO("pendulum length l:%.5f , total mass M: %.5f\n",lc,mass_total);

}
void Robot_Class::Inverse_kinematic_solver2(float target_h)
{
    //进行高度的上下界判断
    double h;
    h = target_h > MAX_TARGET_H ? MAX_TARGET_H : target_h;
    h = h < MIN_TARGET_H ? MIN_TARGET_H : h;

    //进行是否进行了初始化的判断
    if(is_robot_init!=1)
        return; //没有初始化完毕，不允许求解逆运动学。
    //下述内容对逆运动学进行计算


    double A,B;
    A = l1*(mass_total - 0.5*mass_both_thigh - mass_both_shank);
    B = l2*(0.5*mass_both_shank - mass_total);
    
    double a,b,c; //一元二次函数的参数;
    a = ((A*l2/B)*(A*l2/B) - l1*l1);
    b =  2*h*l1;
    c = l2*l2 - h*h  - (A*l2/B)*(A*l2/B);
    // ROS_INFO("Solve a: %.3f b: %.3f c: %.3f",a,b,c);

    double sin_a,sin_b;
    sin_a = (-b + sqrt(b*b - 4*a*c))/(2*a); 
    if(sin_a <-1 || sin_a > 1){ //判断结果是否在[-1 1]之间，根据几何意义，不会存在两个解都在此区间的情况
        sin_a = (-b - sqrt(b*b - 4*a*c))/(2*a); 
    }
    //求解结果转换为关节参数进行下达
    double alpha =  asin(sin_a); //

    double beta = acos( A * cos(alpha) / B );
    target_hip_pos_k = (0.5* 3.1415926) - (float)(alpha);

    target_knee_pos_k = -(beta - alpha) ;

    double error =  h - (l1*cos(target_hip_pos_k) + l2*cos(target_hip_pos_k + target_knee_pos_k));
    ROS_INFO("Solve sin_a: %.3f  theta1: %.3f theta2: %.3f, error: %.3f",sin_a,target_hip_pos_k,target_knee_pos_k,error);
  
    
    // 出于设计LQR控制器需要，得计算一下模拟倒立摆的杆长
    float lc;
    float real_h;

    real_h = l1*cos(target_hip_pos_k) + l2*cos(target_hip_pos_k + target_knee_pos_k);

    lc = real_h - (mass_both_thigh*0.5*l1*cos(target_hip_pos_k) +
     mass_both_shank*(l1*cos(target_hip_pos_k) + 0.5*l2*cos(target_hip_pos_k + target_knee_pos_k)) + 
     0)/mass_total;


    ROS_INFO("pendulum length l:%.5f , total mass M: %.5f\n",lc,mass_total);

}





#endif 