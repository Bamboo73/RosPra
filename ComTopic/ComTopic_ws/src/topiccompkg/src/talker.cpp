#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>



//【大概熟悉一个节点是如何定义一个talker来talk消息的就可以，不需要记住。
//【不参考内容的情况下编写下列代码的能力，我的评价是 可以 但 没 必 要
int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");//【这样就可以写中文了】

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"talker"); // ZZW如果使用launch文件启动node，node名称会被覆写，这一点要注意。
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能
    //ZZW：句柄是需要定义的，不然使用rostopic list 在控制台中可能无法看到“talker”这个node。


    //4.实例化 发布者 对象【】
    //泛型: 发布的消息类型  【什么是泛型？】
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter",10);



    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    std_msgs::String msg; //ZZW: "::"运算符表示所属关系，对于OO编程而言，相当于说定义一个String类型变量
    // Z| ： 并且这个String变量是定义在std_msgs这个类当中的。
    // msg.data = "你好啊！！！";
    std::string msg_front = "Hello 你好！"; //消息前缀
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(1);

    //节点不死【不过从道理上来说，一个程序能判断自己死没死也挺有意思的... ...】
    //【我的理解是，总的大程序在跑的时候会看自己所 是 的哪个节点在整个环境中还在不在】
    //【不在的话就不执行while了】
    //【所以从分层的角度而言这里也没有什么语义或逻辑错误，没有否定的自我指涉】
    while (ros::ok())
    {
        //使用 stringstream 拼接字符串与编号
        std::stringstream ss;
        ss << msg_front << count;
        msg.data = ss.str();
        //发布消息
        pub.publish(msg);
        //加入调试，打印发送的消息
        ROS_INFO("发送的消息:%s",msg.data.c_str());

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
    }


    return 0;
}