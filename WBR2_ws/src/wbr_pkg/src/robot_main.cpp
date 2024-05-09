#include "wbr_pkg/misc.h" // 机器人的结构体定义在misc.h文件当中
#include "wbr_pkg/pid.h"
#include "wbr_pkg/dpid_ctrl.h"
#include "ros/ros.h"

Robot_Class Myrobot; //不知道为什么会标红... ... 在misc.h 里面写的很清楚了我认为。


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


int main(int argc, char * argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Robot");
    ros::NodeHandle nt; 

    Myrobot.InitRobot(); // 初始化机器人对象
    ros::Subscriber ReadImuValue = nt.subscribe<sensor_msgs::Imu>("imu",10,doImuMsg);



    ROS_INFO("Robot is on");
    return 0;
}