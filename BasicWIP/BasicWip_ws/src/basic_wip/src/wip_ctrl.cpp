#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include "geometry_msgs/Twist.h"

#include "math.h"

//尝试读取机器人轮子的位置信息，然后实现一个针对位置计算速度的观测器。
// 可以尝试和直接读取的vel进行对比，看看这个观测器写的对不对。
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"



//定义一个机器人Class ，包含关于运动当中的各种所需属性。
class Robot_Class{
    public:
        //所有变量使用小写打头，下划线|非个单词的大驼峰来命名。
        float orientation_yaw; //rad
        float orientation_pitch; //rad
        float orientation_roll; //rad

        float balance_pid_kp;
        float balance_pid_ki;
        float balance_pid_kd;
        float balance_last_error;
        float balance_target;  // rad
        float balance_calculated_effort; // N`m
        bool  balancePidFirstFlag;


        float LeftJoint_Pos;
        float LeftJoint_Vel;
        float LeftJoint_Trq;    
        float RightJoint_Pos;    
        float RightJoint_Vel;
        float RightJoint_Trq;


        void InitRobot() // 初始化机器人函数。主要是控制参数等。
        {
            balancePidFirstFlag = 1;
            balance_target = 0.0;
            balance_calculated_effort = 0.0;
        }
        void SetRobotBalancePidParams(float kp,float ki,float kd); //后文实现
        void BalancePidControl();
        void SetRobotBalanceCmd(float LeftEcmd, float RightEcmd); //只是用来发布控制消息的，解耦用。
        void SetRobotOrientations(float yaw,float pitch, float roll); // 后文实现

};
void Robot_Class::SetRobotBalancePidParams(float kp,float ki,float kd)
{
    balance_pid_kp = kp;
    balance_pid_ki = ki;
    balance_pid_kd = kd;
}
void Robot_Class::BalancePidControl()
{
    float error = balance_target - orientation_pitch;
    if(balancePidFirstFlag == 1) //PID是否为初次的判断，感觉还是有必要的，如果只是简单的将lasterro=0，那么第一次计算微分项可能导致一个大脉冲，不好。
        {
            balance_last_error = error;
            balancePidFirstFlag = 0;
        }
    balance_calculated_effort = balance_pid_kp * (balance_target - orientation_pitch) + balance_pid_kd * (error - balance_last_error);
    balance_last_error = error;
}
void Robot_Class::SetRobotOrientations(float yaw,float pitch,float roll)
{
    orientation_pitch = pitch;
    orientation_roll = roll;
    orientation_yaw = yaw;
}
void Robot_Class::SetRobotBalanceCmd(float LeftEcmd, float RightEcmd) //只是用来发布控制消息的，解耦用。
{
    //LeftWheelEffortCmdPub.publish(LeftEcmd);
    //RightWheelEffortCmdPub.publish(RightEcmd);
}
Robot_Class Myrobot;

void doMsg(const  sensor_msgs::Imu::ConstPtr & msg_p){  // 传感器的回调函数
    //ROS_INFO("Imu Signal Received.");
    float w,x,y,z;
    w = msg_p->orientation.w;
    x = msg_p->orientation.x;
    y = msg_p->orientation.y;
    z = msg_p->orientation.z;

    float psi,theta,phi;
    psi = atan2( 2*(w*x + y*z), 1- 2*(x*x  + y*y)); //roll
    theta = asin(2*(w*y -z*x));                     //pitch
    phi = atan2(2*(w*z + x*y),1-2*(y*y + z*z));     //yaw
   
    Myrobot.orientation_yaw = phi;
    Myrobot.orientation_pitch = theta;
    Myrobot.orientation_roll = psi;

}

void doMsg2(const  sensor_msgs::JointState::ConstPtr & joint_msg_p){  // 传感器的回调函数

    Myrobot.LeftJoint_Pos = joint_msg_p->position[0];
    Myrobot.LeftJoint_Vel = joint_msg_p->velocity[0];
    Myrobot.LeftJoint_Trq = joint_msg_p->effort[0];

    Myrobot.RightJoint_Pos = joint_msg_p->position[1];
    Myrobot.RightJoint_Vel = joint_msg_p->velocity[1];
    Myrobot.RightJoint_Trq = joint_msg_p->effort[1];

}

int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Controller");
    ros::NodeHandle nt; //  创建节点句柄

    Myrobot.InitRobot(); // 初始化机器人对象

    // 控制器相关
    ros::Publisher LeftWheelEffortCmdPub = nt.advertise<std_msgs::Float64>("/robot_wip/Lwheel_effort_controller/command",10);
    ros::Publisher RightWheelEffortCmdPub = nt.advertise<std_msgs::Float64>("/robot_wip/Rwheel_effort_controller/command",10);
    

    // 反馈相关
    ros::Subscriber ReadJointValue = nt.subscribe<sensor_msgs::JointState>("/robot_wip/joint_states",10,doMsg2);
    // ros::Publisher RightWheelPosPub = nt.advertise<std_msgs::Float64>("/zzw/right_wheel_pos",10);
    // ros::Publisher RightWheelVelPub = nt.advertise<std_msgs::Float64>("/zzw/right_wheel_vel",10);
    ros::Publisher RightWheelTrqPub = nt.advertise<std_msgs::Float64>("right_wheel_trq",10);
    // ros::Publisher LeftWheelPosPub = nt.advertise<std_msgs::Float64>("/zzw/left_wheel_pos",10);
    // ros::Publisher LeftWheelVelPub = nt.advertise<std_msgs::Float64>("/zzw/left_wheel_vel",10);
    ros::Publisher LeftWheelTrqPub = nt.advertise<std_msgs::Float64>("left_wheel_trq",10);
    
    // IMU 相关：
    ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doMsg);
    //用于监视倾斜角度的话题 /robot_wip/posture_pitch
    //使用图标读取即可
    ros::Publisher RobotPitchAngle = nt.advertise<std_msgs::Float64>("/robot_wip/posture_pitch",10);
    //ros::Publisher RollSensorPub = nt.advertise<std_msgs::Float64>("/zzw/imu_roll",10); //原来之前写过了（流汗黄豆）
    





    // ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doMsg);
    std_msgs::Float64 BalanceCmdMsg; //控制指令
    std_msgs::Float64 RobotPitch;   //解算后的角度
    std_msgs::Float64 Left_Trq_Msg; //准备上报力矩
    std_msgs::Float64 Right_Trq_Msg;

    //std_msgs::Float64 RollSensorMsg; // 用于获取imu 的roll消息，便于使用rqt_graph 显示。

    Myrobot.SetRobotBalancePidParams(0.23,0,0.2); // 初始化一下PID参数

    ros::Rate r(100);
    
    int control_rate_scaler = 5; //对香农采样定律保持尊敬www。控制的频率控制在传感器频率的0.5倍以下 这里是0.2
    int control_rate_counter = 0; 

    ROS_INFO("talker is on");
    while (ros::ok())
    {
        // ROS_INFO("talker is working");
        control_rate_counter++;

        if(control_rate_counter >= control_rate_scaler)
        {
            Myrobot.BalancePidControl();
            //Myrobot.SetRobotBalanceCmd(-Myrobot.balance_calculated_effort,-Myrobot.balance_calculated_effort);
            BalanceCmdMsg.data = -static_cast<double>(Myrobot.balance_calculated_effort);
            
            //发送控制指令。
            LeftWheelEffortCmdPub.publish(BalanceCmdMsg);
            RightWheelEffortCmdPub.publish(BalanceCmdMsg);


            // pubcmd.publish(tcmd);
            control_rate_counter = 0;
        }   

        // IMU消息解算
        RobotPitch.data = -static_cast<double>(Myrobot.orientation_pitch);
        RobotPitchAngle.publish(RobotPitch);// 用于监视的话题。

        Left_Trq_Msg.data = static_cast<double>(Myrobot.LeftJoint_Trq);
        Right_Trq_Msg.data = static_cast<double>(Myrobot.RightJoint_Trq);
        
        LeftWheelTrqPub.publish(Left_Trq_Msg);
        RightWheelTrqPub.publish(Right_Trq_Msg);
        

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}