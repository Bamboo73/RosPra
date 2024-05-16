#include "wbr_pkg/misc.h" // 机器人的结构体定义在misc.h文件当中
#include "wbr_pkg/pid.h"
#include "wbr_pkg/lqr_ctrl.h"
// #include "ros/ros.h"
// #include "std_msgs/Float64.h"
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
PID_Class Vel_Pid;

Lqr_Class Balance_Lqr;
bool Ctrl_mode = 1;
bool Is_Lqr_Right = 1;


//以下是机器人发送的话题以及Msg，全部定义成全局变量，并且用Cmd称呼，而非使用Pos or trq 使用的时候记得区分。
ros::Publisher LeftWheelCmdPub; 
ros::Publisher RightWheelCmdPub; 
std_msgs::Float64 LeftWheelCmdMsg;
std_msgs::Float64 RightWheelCmdMsg;
ros::Publisher LeftHipCmdPub; 
ros::Publisher RightHipCmdPub; 
ros::Publisher LeftKneeCmdPub;
ros::Publisher RightKneeCmdPub;
std_msgs::Float64 LeftHipCmdMsg;
std_msgs::Float64 RightHipCmdMsg;
std_msgs::Float64 LeftKneeCmdMsg;
std_msgs::Float64 RightKneeCmdMsg;
void RobotCmdSending(float Lhip, float Rhip , float Lknee , float Rknee, float Lwheel,float Rwheel); 
//这个函数的入口是6个关节的float类型，但是不说明是pos还是trq，使用的时候记得说明。


//以下是机器人监视所用到的各种发布者和消息。
ros::Publisher RobotPitchAngle; 
ros::Publisher RobotYawAngle; 
std_msgs::Float64 Robot_Pitch_Ob;
std_msgs::Float64 Robot_Yaw_Ob;

ros::Publisher BalancePidCmdObPub;
ros::Publisher BalanceLqrCmdObPub;
std_msgs::Float64 BalancePidCmdObMsg;
std_msgs::Float64 BalanceLqrCmdObMsg;

ros::Publisher RobotStatesPub;// = nh.advertise<std_msgs::String>("RobotStates",10);
std_msgs::String RobotStatesMsg;
void RobotStatesSending();


void RobotObSending();


void doGazeboLinkMsg(const gazebo_msgs::LinkStates::ConstPtr & msg)
{
    int modelCount = msg->name.size();
    float posr,posl;
    float velr = 0,vell = 0;
    for(int modelInd = 0; modelInd < modelCount; ++modelInd)
    {
        if(msg->name[modelInd] == "wbr_robot::Left_wheel_link")
        {
            posl = msg->pose[modelInd].position.x;
            vell = msg->twist[modelInd].linear.x;
        }
        if(msg->name[modelInd] == "wbr_robot::Right_wheel_link")
        {
            posr = msg->pose[modelInd].position.x;
            velr = msg->twist[modelInd].linear.x;
        }
    }

    Myrobot.robot_dynamic_states[0] = (posr + posl)/2.0;
    Myrobot.robot_dynamic_states[2] = (vell + velr)/2.0;
    // ROS_INFO("direct get vel dx: %.3f",0.5*(velr+ vell));

}

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

    Myrobot.robot_dynamic_states[1] = theta;
    Myrobot.robot_dynamic_states[3] = msg_p->angular_velocity.y;
}

void DynamicPara(wbr_pkg::robotwbrConfig& config, uint32_t level){
    Balance_Pid.kd = config.pos_kd;
    Balance_Pid.kp = config.pos_kp;
    Balance_Pid.ki = config.pos_ki;


    Balance_Lqr.K_Matrix[0] = config.k1;
    Balance_Lqr.K_Matrix[1] = config.k2;
    Balance_Lqr.K_Matrix[2] = config.k3;
    Balance_Lqr.K_Matrix[3] = config.k4;

    Ctrl_mode = config.pid_ctrl;
    Is_Lqr_Right = config.lqr_k_right;
}



int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Robot");
    ros::NodeHandle nh;

    //这里是关于动态调参的内容
        dynamic_reconfigure::Server<wbr_pkg::robotwbrConfig> server;
        dynamic_reconfigure::Server<wbr_pkg::robotwbrConfig>::CallbackType cbType;
        cbType = boost::bind(&DynamicPara,_1,_2);
        //服务器对象调用回调对象
        server.setCallback(cbType);  
    //动态调参Over

    Myrobot.InitRobot(); // 初始化机器人对象
    Myrobot.SetJointCmdSendFunctionPtr(RobotCmdSending); //绑定函数指针，指向上述函数。

    //IMU
    ros::Subscriber ReadImuValue = nh.subscribe<sensor_msgs::Imu>("imu",10,doImuMsg);
    ros::Subscriber ReadGazeboLinkValues = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",10,doGazeboLinkMsg);
        
    //所有和主动发送机器人状态有关的节点，统一在/robot_observe之下
    RobotPitchAngle = nh.advertise<std_msgs::Float64>("/robot_ob/posture_pitch",10);
    RobotYawAngle = nh.advertise<std_msgs::Float64>("/robot_ob/posture_yaw",10);
    BalancePidCmdObPub = nh.advertise<std_msgs::Float64>("/robot_ob/pid_cmd",10);
    BalanceLqrCmdObPub = nh.advertise<std_msgs::Float64>("/robot_ob/lqr_cmd",10);


    RobotStatesPub = nh.advertise<std_msgs::String>("RobotStates",10);

    // 关节控制准备发布
    // 注：下述内容定义为全局变量。
    LeftWheelCmdPub  = nh.advertise<std_msgs::Float64>("/robot_wbr/Left_wheel_ctrl/command",10);
    RightWheelCmdPub = nh.advertise<std_msgs::Float64>("/robot_wbr/Right_wheel_ctrl/command",10);    
    LeftHipCmdPub    = nh.advertise<std_msgs::Float64>("/robot_wbr/Left_hip_ctrl/command",10);
    RightHipCmdPub   = nh.advertise<std_msgs::Float64>("/robot_wbr/Right_hip_ctrl/command",10);
    LeftKneeCmdPub   = nh.advertise<std_msgs::Float64>("/robot_wbr/Left_knee_ctrl/command",10);
    RightKneeCmdPub  = nh.advertise<std_msgs::Float64>("/robot_wbr/Right_knee_ctrl/command",10);

    

    // ----------------------------------定义各种发布与订阅完毕----------------------------------
    // ----------------------------------准备进行轮循前运行----------------------------------
    Balance_Pid.Init_pid();
    Balance_Pid.Set_target_value(0);
    Balance_Pid.Set_pid_param(9,0,10,-1.57,1.57); // 测试得到的PID 9 0 10,积分上下限就先给到两个90°好了

    // ----------------------------------准备进入轮循----------------------------------   
    ros::Rate r(LoopRate); // 25Hz
    while (ros::ok())
    {
        //计算得到两个轮子的驱动力矩。
        float temp_pid = -Balance_Pid.Calculate_pid_output(Myrobot.pitch);
        float temp_lqr;
            if(Is_Lqr_Right == 1)
                temp_lqr = Balance_Lqr.LqrCalculate(Myrobot.robot_dynamic_states);
            else if (Is_Lqr_Right == 0)
                temp_lqr = -Balance_Lqr.LqrCalculate(Myrobot.robot_dynamic_states);

        BalanceLqrCmdObMsg.data = temp_lqr;
        if(Ctrl_mode == 1){
            Myrobot.target_left_wheel_trq = temp_pid;
            Myrobot.target_right_wheel_trq = Myrobot.target_left_wheel_trq;
        }
        else if(Ctrl_mode == 0){
            Myrobot.target_left_wheel_trq = temp_lqr;
            Myrobot.target_right_wheel_trq = Myrobot.target_left_wheel_trq;
        }
        Myrobot.target_left_hip_pos_k = 0.367;
        Myrobot.target_right_hip_pos_k = 0.367;
        Myrobot.target_left_knee_pos_k = -0.641;
        Myrobot.target_right_knee_pos_k = -0.641;

        RobotStatesSending();   
        RobotObSending();
        Myrobot.CallJointCmdSend(Myrobot.target_left_hip_pos_k, 
                        Myrobot.target_right_hip_pos_k,
                        Myrobot.target_left_knee_pos_k,
                        Myrobot.target_right_knee_pos_k,
                        Myrobot.target_left_wheel_trq, 
                        Myrobot.target_right_wheel_trq 
        );

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}


// 机器人控制指令的发送函数，可以通过Robot_Class对象的函数指针进行指向。
void RobotCmdSending(float Lhip, float Rhip , float Lknee , float Rknee, float Lwheel,float Rwheel)
{
    //两轮控制指令发送 
    LeftWheelCmdMsg.data = Lwheel;
    RightWheelCmdMsg.data = Rwheel;

    
    LeftWheelCmdPub.publish(LeftWheelCmdMsg);
    RightWheelCmdPub.publish(RightWheelCmdMsg);

    LeftHipCmdMsg.data = Lhip;
    RightHipCmdMsg.data = Rhip;
    LeftHipCmdPub.publish(LeftHipCmdMsg);
    RightHipCmdPub.publish(RightHipCmdMsg);

    LeftKneeCmdMsg.data = Lknee;
    RightKneeCmdMsg.data = Rknee;
    LeftKneeCmdPub.publish(LeftKneeCmdMsg);
    RightKneeCmdPub.publish(RightKneeCmdMsg);
}

void RobotStatesSending()
{
    std::stringstream ss;
    ss << " phi:" << Myrobot.robot_dynamic_states[0];
    ss << " theta:" << Myrobot.robot_dynamic_states[1];
    ss << " d_phi:" << Myrobot.robot_dynamic_states[2];
    ss << " d_theta:" << Myrobot.robot_dynamic_states[3];

    RobotStatesMsg.data = ss.str();
    RobotStatesPub.publish(RobotStatesMsg);
}


void RobotObSending()
{
    Robot_Pitch_Ob.data = Myrobot.pitch;
    Robot_Yaw_Ob.data = Myrobot.yaw;

    BalancePidCmdObMsg.data = Myrobot.target_left_wheel_trq;
    //BalanceLqrCmdObMsg.data = Balance_Lqr.LqrCalculate(Myrobot.robot_dynamic_states);


    RobotPitchAngle.publish(Robot_Pitch_Ob);
    RobotYawAngle.publish(Robot_Yaw_Ob);
    BalancePidCmdObPub.publish(BalancePidCmdObMsg);
    BalanceLqrCmdObPub.publish(BalanceLqrCmdObMsg);
}