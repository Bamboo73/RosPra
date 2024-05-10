#include "wbr_pkg/misc.h" // 机器人的结构体定义在misc.h文件当中
#include "wbr_pkg/pid.h"
#include "wbr_pkg/dpid_ctrl.h"
#include "ros/ros.h"

//动态调参
#include "dynamic_reconfigure/server.h"


// #include "wbrtestpkg/drConfig.h"
// #include 

// 进行一些获取轮子位置的尝试：
// #include <tf/transform_listener.h>
#include "gazebo_msgs/LinkStates.h"
// devel/include/basic_lqr_pkg/basiclqrConfig.h
#include "wbr_pkg/robotwbrConfig.h"


#define LoopRate 25 //HZ 


Robot_Class Myrobot; //不知道为什么会标红... ... 在misc.h 里面写的很清楚了我认为。
PID_Class Balance_Pid;

void doImuMsg(const  sensor_msgs::Imu::ConstPtr & msg_p){  // 传感器的回调函数
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
   
    Myrobot.yaw = phi;
    Myrobot.pitch = theta;
    Myrobot.roll = psi;

}

void DynamicPara(wbr_pkg::robotwbrConfig& config, uint32_t level)
{
    Balance_Pid.kd = config.pos_kd;
    Balance_Pid.kp = config.pos_kp;
    Balance_Pid.ki = config.pos_ki;

}



int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Robot");
    ros::NodeHandle nt;
    //这里是关于动态调参的内容
    dynamic_reconfigure::Server<wbr_pkg::robotwbrConfig> server;
    dynamic_reconfigure::Server<wbr_pkg::robotwbrConfig>::CallbackType cbType;
    cbType = boost::bind(&DynamicPara,_1,_2);
    // 5.服务器对象调用回调对象
    server.setCallback(cbType);  


    Myrobot.InitRobot(); // 初始化机器人对象
    
    //IMU
    ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doImuMsg);    
    //所有和主动发送机器人状态有关的节点，统一在/robot_observe之下
    ros::Publisher RobotPitchAngle = nt.advertise<std_msgs::Float64>("/robot_ob/posture_pitch",10);
    ros::Publisher RobotYawAngle = nt.advertise<std_msgs::Float64>("/robot_ob/posture_yaw",10);
    std_msgs::Float64 Robot_Pitch_Ob;
    std_msgs::Float64 RobotWheelVelOb;

    // 关节控制准备发布
    ros::Publisher LeftWheelCmdPub = nt.advertise<std_msgs::Float64>("/robot_wbr/Left_wheel_ctrl/command",10);
    ros::Publisher RightWheelCmdPub = nt.advertise<std_msgs::Float64>("/robot_wbr/Right_wheel_ctrl/command",10);
    std_msgs::Float64 LeftWheelCmdMsg;
    std_msgs::Float64 RightWheelCmdMsg;
    float Trq_cmd;

    // ----------------------------------定义各种发布与订阅完毕----------------------------------
    // ----------------------------------准备进行轮循前运行----------------------------------
    Balance_Pid.Init_pid();
    Balance_Pid.Set_target_value(0);
    // Balance_Pid.Set_pid_param();


    // ----------------------------------准备进入轮循----------------------------------   
    ros::Rate r(LoopRate);
    while (ros::ok())
    {
        Trq_cmd = -Balance_Pid.Calculate_pid_output(Myrobot.pitch);
        LeftWheelCmdMsg.data = Trq_cmd;
        RightWheelCmdMsg.data = Trq_cmd;
        LeftWheelCmdPub.publish(LeftWheelCmdMsg);
        RightWheelCmdPub.publish(RightWheelCmdMsg);

        Robot_Pitch_Ob.data = Myrobot.pitch;
        RobotPitchAngle.publish(Robot_Pitch_Ob);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}