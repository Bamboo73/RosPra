/*
 * 双足轮退机器人的控制代码，给予倒立摆模型设计。
 * 机器人约束如下：
 * 1. 机器人的两腿同步运动。 （左右膝关节；左右髋关节；使用相同指令）
 * 2. 倒立摆模型，并且有：机器人的髋关节、膝关节使用位置控制，根据高度解算位置。
 * 3. 双环pd控制器：
 *          - 平衡pd控制：输入角度，输出力矩；
 *          - 速度pd控制：输入参考速度，输出角度；
 * 
*/
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

//部署传感器
#include "sensor_msgs/Imu.h"


//动态调参
#include "dynamic_reconfigure/server.h"


// #include "wbrtestpkg/drConfig.h"
// #include 

// 进行一些获取轮子位置的尝试：
// #include <tf/transform_listener.h>
#include "gazebo_msgs/LinkStates.h"
// devel/include/basic_lqr_pkg/basiclqrConfig.h
#include "basic_lqr_pkg/basiclqrConfig.h"




#define LoopRate 100
#define CtrlScaler 5
#define winlength_max 50

//定义一个机器人Class ，包含关于运动当中的各种所需属性。
class Robot_Class{
    public:
        bool   robot_init_flag = 0;
        double mass_body;
        double mass_both_thigh;
        double mass_both_shank;
        double mass_total;

        double l1 = 0.3;
        double l2 = 0.3;
        double target_h; // 机器人期望高度，从轮心到平台中心
        double wheel_radius;

        //所有变量使用小写打头，下划线|非个单词的大驼峰来命名。
        float orientation_yaw; //rad
        float orientation_pitch; //rad
        float orientation_roll; //rad

        //平衡控制用到的属性
        float balance_pid_kp;
        float balance_pid_ki;
        float balance_pid_kd;
        float balance_last_error = 0;
        float balance_target;  // rad
        float balance_calculated_effort; // N`m
        bool  balancePidFirstFlag; //是否首次开启平衡控制。

        //速度控制用到的属性
        float vel_pid_kp;
        float vel_pid_ki;
        float vel_pid_kd;
        float vel_pid_last_error = 0;
        float vel_error_sum = 0;
        float vel_target;  // rad
        float vel_pid_calculated_pos; // rad
        bool  velPidFirstFlag; //是否首次开启速度控制。

        //速度闭环需要的参数：
        //以下四者将直接从仿真环境中读取
        double l_wheel_pos;
        double l_wheel_pos_last;
        double r_wheel_pos_last;
        double r_wheel_pos;

        double l_wheel_vel_get; //所谓的get是直接从仿真环境中读取关节速度。
        double r_wheel_vel_get;
        
        //以下两者将通过“编码器解算”得到。
        bool   vel_cal_flag; //在机器人成功读取位置之后，才能计算速度。
        double l_wheel_vel_cal; //对应的cal是通过轮子的位置，结合采样周期求解得到的。
        double r_wheel_vel_cal;
        double l_wheel_filter[winlength_max] = {0}; //窗口长度为10的滤波器，就用求平均值的方式进行滤波
        double r_wheel_filter[winlength_max] = {0};
        int winlength;

        //机器人上肢关节控制量的目标值（平衡控制在双环PD内部已经有了，见上面的部分）
        float Target_Pos_Hip;
        float Target_Pos_Knee;

        //机器人的状态变量们。
        float X_states[4];          // 机器人当前状态 x q dx dq;

        void InitRobot() // 初始化机器人函数。主要是控制参数等。
        {
            balancePidFirstFlag = 1;
            balance_target = 0.0;
            balance_calculated_effort = 0.0;


            winlength = 10; //速度滤波器长度设置。
            velPidFirstFlag =1;
            vel_target = 0.0;
            vel_pid_calculated_pos = 0.0;
            vel_error_sum = 0;

            // mass info
            mass_body = 8.1339;
            mass_both_thigh = 0.43749 * 2.0;
            mass_both_shank = 0.44105 * 2.0;
            mass_total = mass_body + mass_both_shank + mass_both_thigh;
            // length info
            l1 = 0.3;
            l2 = 0.3;
            target_h = 0.5;
            
            wheel_radius = 0.05; //0.05
            
            vel_cal_flag=0; //一开始不允许计算速度（因为没有一开始的位置）

            robot_init_flag = 1; //The robot is initialized.
            
        }
        void InverseKinematicsSolving(double target_h); // 求解逆运动学

        void SetRobotBalancePidParams(float kp,float ki,float kd); //后文实现
        void BalancePidControl();

        void SetRobotVelPidParams(float kp,float ki,float kd);
        void VelPidControl();
        void VelSpeedCalculation(double delta_t);


        void LQR_Contorller();


        void SetRobotBalanceCmd(float LeftEcmd, float RightEcmd); //只是用来发布控制消息的，解耦用。
        void SetRobotOrientations(float yaw,float pitch, float roll); // 后文实现

};

// 机器人关节角度的计算（结合高度和质心位置求解）
void Robot_Class::InverseKinematicsSolving(double target_h)
{
    //进行高度的上下界判断
    double h;
    h = target_h>0.55?0.55:target_h;
    h = h<0.1?0.1:h;

    //进行是否进行了初始化的判断
    if(robot_init_flag!=1)
        return; //没有初始化完毕，不允许求解逆运动学。
    //下述内容对逆运动学进行计算
    double A,B;
    A = l1 * (mass_total - 0.5*mass_both_thigh - mass_both_shank) / mass_total;
    B = l2 * (0.5 * mass_both_shank - mass_total) / mass_total;
    // ROS_INFO("Solve A: %.3f B: %.3f \n",A,B);
    double a,b,c; //一元二次函数的参数;
    a = (l1*l1)/(l2*l2) - A*A/(B*B);
    b = - 2*target_h*l1/ (l2*l2);
    c = target_h*target_h/(l2*l2) + A*A/(B*B) - 1;

    // ROS_INFO("Solve a: %.3f b: %.3f c: %.3f\n",a,b,c);
    double cosa,cosb;
    cosa = (-b + sqrt(b*b - 4*a*c))/2*a; //只选择正解，要求theta1 在pi/2以下。
    //求解结果转换为关节参数进行下达
    Target_Pos_Hip = (float)acos(cosa);
    double beta = asin( A * sin(Target_Pos_Hip) / B );
    Target_Pos_Knee = beta - Target_Pos_Hip;

    // ROS_INFO("Solve cosa: %.3f  theta1: %.3f theta2: %.3f\n",cosa,Target_Pos_Hip,Target_Pos_Knee);
    
    // 出于设计LQR控制器需要，得计算一下模拟倒立摆的杆长
    float lc;
    float y_c;
    y_c = (-mass_both_thigh*(0.5* l1 * cosa ) - (l1* cosa + 0.5*l2*cos(beta))*mass_both_shank)/mass_total;
    lc = h+y_c;
    // ROS_INFO("pendulum length l:%.5f , total mass M: %.5f",lc,mass_total);

}

//设置平衡PID控制的参数
void Robot_Class::SetRobotBalancePidParams(float kp,float ki,float kd)
{
    balance_pid_kp = kp;
    balance_pid_ki = ki; 
    balance_pid_kd = kd;
}

//求解平衡控制的输出量（电机力矩）
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

//设置速度PID控制的参数
void Robot_Class::SetRobotVelPidParams(float kp,float ki,float kd)
{
    vel_pid_kp = kp;
    vel_pid_ki = ki;
    vel_pid_kd = kd;
}

void Robot_Class::VelPidControl() 
{
    //定义一个饱和特性，不允许机器人的目标位置太离谱。
    float max_pos = 1.3;
    float min_pos = -1.3;



    float error = vel_target - (float)((l_wheel_vel_cal + r_wheel_vel_cal)/2.0);
    if(velPidFirstFlag == 1) //PID是否为初次的判断，感觉还是有必要的，如果只是简单的将lasterro=0，那么第一次计算微分项可能导致一个大脉冲，不好。
    {
        vel_pid_last_error = error;
        velPidFirstFlag = 0;
        vel_error_sum = 0;
    }
    vel_pid_calculated_pos = vel_pid_kp * error + vel_pid_ki*vel_error_sum + vel_pid_kd * (error - vel_pid_last_error);
    
    balance_target = vel_pid_calculated_pos>max_pos?max_pos:vel_pid_calculated_pos;
    balance_target = balance_target<min_pos?min_pos:balance_target;
    // ROS_INFO("Target theta: %.2f",balance_target);
    vel_pid_last_error = error;
    vel_error_sum += error;
}

void Robot_Class::VelSpeedCalculation(double delta_t) //这下有滤波器了
{
    if(vel_cal_flag != 1)
        return ;

    // ROS_INFO("delta_t is: %.2f",delta_t);
    int i;
    
    //求解一下位置。[LQR 相关] 【已经弃用】
    // X_states[0] = wheel_radius * 0.5 *(r_wheel_pos + l_wheel_pos);

    double filtersum[2]; //存储左右滤波器的中途求和 0:left 1:right
    double r_wheel_pos_error = r_wheel_pos - r_wheel_pos_last;
    double l_wheel_pos_error = l_wheel_pos - l_wheel_pos_last;

    r_wheel_vel_cal = (r_wheel_pos_error/delta_t)*wheel_radius;
    l_wheel_vel_cal = (l_wheel_pos_error/delta_t)*wheel_radius;

    filtersum[0] = 0.0;
    filtersum[1] = 0.0;
    for (i = 0;i< winlength-1;i++)
    {
        l_wheel_filter[i] = l_wheel_filter[i+1];
        filtersum[0]+=l_wheel_filter[i];

        r_wheel_filter[i] = r_wheel_filter[i+1];
        filtersum[1]+=r_wheel_filter[i];
    }//此时 i = winlength - 1
        l_wheel_filter[i] = l_wheel_vel_cal; //最后一次计算结果。
        filtersum[0]+=l_wheel_filter[i];

        r_wheel_filter[i] = r_wheel_vel_cal; //最后一次计算结果。
        filtersum[1]+=r_wheel_filter[i];
    //此时，filtersum中已经存放了左右速度的 winlength 个累计和。
    l_wheel_vel_cal = filtersum[0] / (1.0*winlength);
    r_wheel_vel_cal = filtersum[1] / (1.0*winlength);


    //LQR有关的内容：
    X_states[2] = (l_wheel_vel_cal + r_wheel_vel_cal)*0.5;

    // ROS_INFO("delta_t is: %.2f; delta_lp is : %.2f ; cal_l_vel is: %.2f", \
    delta_t,l_wheel_pos_error,l_wheel_vel_cal);
    //更新上一次的位置。
    r_wheel_pos_last = r_wheel_pos;
    l_wheel_pos_last = l_wheel_pos;
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

void Robot_Class::LQR_Contorller() //说明：测试中，目前只支持h=0.5 的情况
{
    float X_des[4] = {0,0,0,0}; // 目标位置：别动



   

    float K[4] = {0.100995  , 0.064053  , 0.460320,  0.025315}; //计算得到的状态反馈矩阵

    int i;
    float ctrl_u = 0.0;
    for(i = 0;i<4;i++)
    {
        ctrl_u += K[i]*(X_des[i] - X_states[i]);
    }
    balance_calculated_effort = ctrl_u;
}


Robot_Class Myrobot;

/*
*   结束对机器人类和对象的实现----------------------------------------------------------------------
*/


/*
*   下面开始的是一系列回调函数
*/

//动态调参参数服务器。

void DynamicPara(basic_lqr_pkg::basiclqrConfig& config, uint32_t level)
{
    // ROS_INFO("dynamic paras :%d,%.2f,%d,%s,%d",
    //     config.int_param,
    //     config.double_param,
    //     config.bool_param,
    //     config.string_param.c_str(),
    //     config.list_param
    // );
    Myrobot.balance_pid_kd = config.pos_kd;
    Myrobot.balance_pid_kp = config.pos_kp;
    Myrobot.balance_pid_ki = config.pos_ki;

    Myrobot.vel_pid_kp = config.vel_kp;
    Myrobot.vel_pid_ki = config.vel_ki;
    Myrobot.vel_pid_kd = config.vel_kd;
    
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
   
    Myrobot.orientation_yaw = phi;
    Myrobot.orientation_pitch = theta;
    Myrobot.orientation_roll = psi;

    Myrobot.X_states[1] = theta; // theta = theta;
    Myrobot.X_states[3] = msg_p->angular_velocity.y; // 有点担心对不对... ....

}
void doMsg_height(const  std_msgs::Float64::ConstPtr & msg_p){  // 传感器的回调函数
    double h = msg_p->data;
    h = h>0.55?0.55:h;
    h = h<0.1?0.1:h;

    Myrobot.target_h = h;
}
void doMsg_Vel(const  std_msgs::Float64::ConstPtr & msg_p){  // 传感器的回调函数
    double v = msg_p->data;
    Myrobot.vel_target = v;
}
void doJointStatesMsg(const sensor_msgs::JointState::ConstPtr & msg_p )
{
    Myrobot.l_wheel_pos = msg_p->position[2];
    Myrobot.r_wheel_pos = msg_p->position[5];
    Myrobot.l_wheel_vel_get = msg_p->velocity[2];
    Myrobot.r_wheel_vel_get = msg_p->velocity[5];
    
    if(Myrobot.vel_cal_flag !=1) //这个东西置为1之后，讲道理不应该再乱动了... ... 
    {
        Myrobot.vel_cal_flag = 1;
        Myrobot.r_wheel_pos_last = Myrobot.r_wheel_pos;
        Myrobot.l_wheel_pos_last = Myrobot.l_wheel_pos;
    }
}

void doGazeboLinkMsg(const gazebo_msgs::LinkStates::ConstPtr & msg)
{
    int modelCount = msg->name.size();
    float posr,posl;
    float velr = 0,vell = 0;
    for(int modelInd = 0; modelInd < modelCount; ++modelInd)
    {
        if(msg->name[modelInd] == "test_wip::right_wheel")
        {
            posr = msg->pose[modelInd].position.x;
            velr = msg->twist[modelInd].linear.x;
        }
        if(msg->name[modelInd] == "test_wip::left_wheel")
        {
            posl = msg->pose[modelInd].position.x;
            vell = msg->twist[modelInd].linear.x;
        }
    }

    Myrobot.X_states[0] = (posr + posl)/2.0;
    ROS_INFO("direct get vel dx: %.3f",0.5*(velr+ vell));

}



/*------------------------------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------
 *------------------------------------------------------------------------------------------------
*/
int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Controller");
    ros::NodeHandle nt; //  创建节点句柄

    Myrobot.InitRobot(); // 初始化机器人对象


    // //这里是关于动态调参的内容
    dynamic_reconfigure::Server<basic_lqr_pkg::basiclqrConfig> server;
    dynamic_reconfigure::Server<basic_lqr_pkg::basiclqrConfig>::CallbackType cbType;
    cbType = boost::bind(&DynamicPara,_1,_2);
    // 5.服务器对象调用回调对象
    server.setCallback(cbType);

    //接收的控制消息
    //机器人期望高度
    ros::Subscriber HeightCmdSub = nt.subscribe<std_msgs::Float64>("wbr_height_cmd",10,doMsg_height);
    //机器人期望速度
    ros::Subscriber VelCmdSub = nt.subscribe<std_msgs::Float64>("wbr_vel_cmd",10,doMsg_Vel);

    
    // 左右轮的运动控制
    ros::Publisher LeftWheelCmdPub = nt.advertise<std_msgs::Float64>("/robot_lqr/left_wheel_ctrl/command",10);
    ros::Publisher RightWheelCmdPub = nt.advertise<std_msgs::Float64>("/robot_lqr/right_wheel_ctrl/command",10);
    

    // // 反馈相关
    ros::Subscriber ReadGazeboLinkValues = nt.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",10,doGazeboLinkMsg);
    ros::Subscriber ReadJointValues = nt.subscribe<sensor_msgs::JointState>("/robot_lqr/joint_states",10,doJointStatesMsg);
    // // ros::Publisher RightWheelPosPub = nt.advertise<std_msgs::Float64>("/zzw/right_wheel_pos",10);
    // // ros::Publisher RightWheelVelPub = nt.advertise<std_msgs::Float64>("/zzw/right_wheel_vel",10);
    // ros::Publisher RightWheelTrqPub = nt.advertise<std_msgs::Float64>("right_wheel_trq",10);
    // // ros::Publisher LeftWheelPosPub = nt.advertise<std_msgs::Float64>("/zzw/left_wheel_pos",10);
    // // ros::Publisher LeftWheelVelPub = nt.advertise<std_msgs::Float64>("/zzw/left_wheel_vel",10);
    // ros::Publisher LeftWheelTrqPub = nt.advertise<std_msgs::Float64>("left_wheel_trq",10);
    
    // // IMU 相关：
    ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doImuMsg);

    //所有和主动发送机器人状态有关的节点，统一在/robot_observe之下
    ros::Publisher RobotPitchAngle = nt.advertise<std_msgs::Float64>("/robot_observe/posture_pitch",10);
    ros::Publisher RobotYawAngle = nt.advertise<std_msgs::Float64>("/robot_observe/posture_yaw",10);
    ros::Publisher RobotWheelVel = nt.advertise<std_msgs::Float64>("/robot_observe/wheel_vel",10);
    ros::Publisher RobotWheelVelCal = nt.advertise<std_msgs::Float64>("/robot_observe/wheel_vel_cal",10);
    // //ros::Publisher RollSensorPub = nt.advertise<std_msgs::Float64>("/zzw/imu_roll",10); //原来之前写过了（流汗黄豆）

    // ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doMsg);

    // 控制相关的发布消息
    std_msgs::Float64 BalanceCmdMsg; //控制指令
    std_msgs::Float64 HipPosMsg;
    std_msgs::Float64 KneePosMsg;

    // 监视相关的发布消息
    std_msgs::Float64 RobotPostureOb;
    std_msgs::Float64 RobotWheelVelOb;
    std_msgs::Float64 RobotWheelVelCalOb;


    //std_msgs::Float64 RollSensorMsg; // 用于获取imu 的roll消息，便于使用rqt_graph 显示。
    ros::Rate r(LoopRate); //100Hz

    int control_rate_scaler = CtrlScaler; //对香农采样定律保持尊敬www。控制的频率控制在传感器频率的0.5倍以下 这里是0.2
    int control_rate_counter = 0; //20hz

    int control_vel_rate_counter =0;
    int control_vel_scaler = 4*CtrlScaler;  //5hz 速度调整不应该比角度调整快。这样才可能稳定


    int height_change_rate = 10; //  100/10 = 10hz
    int height_change_counter = 0;
    double height_change_delta = 0.01;
    double height_changing_target;

    ROS_INFO("talker is on");

    //while启动之前的初始化工作
    {
        height_changing_target = 0.5; //默认启动的位置。
        Myrobot.target_h = 0.5;
        Myrobot.balance_target = 0.0;
        Myrobot.vel_target = 0.0;
        //设置一些初始化的参数：
        //平衡PID参数初始状态：
        Myrobot.SetRobotBalancePidParams(3.8,0,5); // 初始化一下PID参数[开启动态调参后这句话就没用了]
        Myrobot.SetRobotVelPidParams(0,0,0);
    }
    float obf;
    // 主循环
    while (ros::ok())
    {
        //逆运动学求解部分，按照频率变化[此举将使机器人关节能够缓慢地变动]
        height_change_counter++;
        if(height_change_counter >= height_change_rate)
        {
            //计算距离给定目标位置的差距
            if(Myrobot.target_h - height_changing_target > height_change_delta)
                height_changing_target += height_change_delta;
            else if(Myrobot.target_h - height_changing_target < -height_change_delta)
                height_changing_target -= height_change_delta;

            //计算逆运动学
            Myrobot.InverseKinematicsSolving(height_changing_target);
            // ROS_INFO("recivedH: %.2f targetH: %.2f theta1: %.2f theta2: %.2f",Myrobot.target_h,height_changing_target, Myrobot.Target_Pos_Hip, Myrobot.Target_Pos_Knee);
            // 发布目标角度位置
            HipPosMsg.data = Myrobot.Target_Pos_Hip;
            KneePosMsg.data = Myrobot.Target_Pos_Knee;


            //清空计时器
            height_change_counter = 0;
        }

        //速度控制部分
        control_vel_rate_counter++;
        if(control_vel_rate_counter >= control_vel_scaler)
        {
            //control_vel_rate_counter = 0;
            // Myrobot.VelSpeedCalculation(); 关于速度的计算放到主循环当中，正好长度是10【越来越屎山了】
            Myrobot.VelPidControl();

            RobotWheelVelCalOb.data = Myrobot.l_wheel_vel_cal;
            RobotWheelVelCal.publish(RobotWheelVelCalOb);
            control_vel_rate_counter = 0;

            //借低频宝地一用
            ROS_INFO("x: %.3f q: %.3f dx: %.3f dq: %.3f \n dpdu: %.3f lqru: %.3f",
            Myrobot.X_states[0],Myrobot.X_states[1],Myrobot.X_states[2],Myrobot.X_states[3]
            ,Myrobot.balance_calculated_effort,obf);
        }

        //平衡控制部分
        control_rate_counter++;
        if(control_rate_counter >= control_rate_scaler)
        {
            //在速度控制实现之前，这里使用的平衡位置为0：
            //Myrobot.balance_target = 0;
            //Myrobot.balance_target = Myrobot.vel_pid_calculated_pos;
            // ROS_INFO("Target theta: %.2f",Myrobot.balance_target);
            //进行一次平衡PID计算
            
            Myrobot.LQR_Contorller();
            obf = Myrobot.balance_calculated_effort;
            
            Myrobot.BalancePidControl();
            //Myrobot.balance_calculated_effort = obf; 
        
            
                
            //在wbrtestpkg这个包里面，下面这句话已经没用了，因为xacro文件我检查过，左右关节是对的。
            //Myrobot.SetRobotBalanceCmd(-Myrobot.balance_calculated_effort,-Myrobot.balance_calculated_effort);
            
            BalanceCmdMsg.data = - static_cast<double>(Myrobot.balance_calculated_effort);
            BalanceCmdMsg.data = 0.0;

            LeftWheelCmdPub.publish(BalanceCmdMsg);
            RightWheelCmdPub.publish(BalanceCmdMsg);

            //清空分频计时器
            control_rate_counter = 0;
        }  


        if(1) // 无论如何都会发布的一些调试性信息
        {
            // 发布机器人倾斜角信息
            RobotPostureOb.data = Myrobot.orientation_pitch; // 姿态监视器先监视俯仰角
            RobotPitchAngle.publish(RobotPostureOb);
            // 发布机器人偏航角度信息
            RobotPostureOb.data = Myrobot.orientation_yaw; // 姿态监视器监视偏航角
            RobotYawAngle.publish(RobotPostureOb);
            
            RobotWheelVelOb.data = Myrobot.l_wheel_vel_get; //用左腿就ok
            RobotWheelVel.publish(RobotWheelVelOb);   

            Myrobot.VelSpeedCalculation( 1.0  /LoopRate);//计算速度（一个长度为winlength的均值滤波器）
            //计算结果将保存在 r|l _wheel_vel_cal 当中
        }

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}