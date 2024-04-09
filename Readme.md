# 关于 ReviewFiles

---

每一次长时间不用Ubuntu上的ROS环境，就会忘记如何使用。之前每一次都新建一个工作空间然后在里面调试，感觉很混乱，这个ReviewFiles可以按照“所学知识内容”建立不同的工作空间，实现了具体的功能之后就另起炉灶，互不干扰，有独立的Readme.md文件。



此库已经建立了VMware中和双系统中的Ubuntu工程文件的跨平台使用。但是可能会有些混乱。

因为ROS中能够搬来搬去的是包，而不是工程文件。

因此，本文件中的工程文件是在双系统中使用的，而工程文件可以搬来搬去。





## assets目录

用来放通目录下md文件图片的。与md文件名解耦，防止git后或者复制后出各种问题。（印象里反正是有问题hhh）

## /BasicStructures

[20240227]

基本的ROS代码架构，不用VSCODE实现；

如果失忆了，完全不记得ros文件架构了，看看这个。（233）

【关于节点命名的AT】

## /BasicVSCode

[20240227]

基本的ROS代码架构，使用VSCODE实现；

使用launch文件。

【关于.vscode文件的说明】

【关于launch文件中标签的说明】

## /CmdTopic

[20240228]

包含两个pkg。

第一个pkg实现了两个node之间的同一个topic的发送和接受；

第二个pkg实现了编写node发送消息控制turtlesim的小乌龟运动。

【关于ros::spinOnce()和ros::spin()的说明】

## /BasicUrdf

[20240229]

包含两个pkg。

basic_urdf_pkg:基本的urdf使用方式与launch启动方式。

basic_xacro_pkg：基本的xacro使用方式与launch启动方式。

【关于 joint_states  robot_states 的 ***提 及***（没有详细研究这些pkgs和nodes）】

## /BasicGazebo

[20240229]

一个pkg。两个部分。

robot1.urdf/launch: 基本的在gazebo环境中载入一个模型。

robot2.launch + /xacro2 : 在gazebo环境中（含有障碍物的环境）载入一个xacro描述的复杂模型。

【gazebo载入模型所涉及的launch xml代码还是值得看一看的】

## /BasicWIP

[20240302]

一个pkg，两个部分。

robot1. xacro /robot1.launch 在gazebo中载入wip模型（不含控制器、驱动器、传感器）

robot2 xacro/yaml/launch | wip_ctrl.cpp (wip_ctrl_node) 在gazebo中载入wip ，绑定控制器(底层transmission & ros_control) 与传感器、编写控制逻辑[通过topic给下层controller发送控制指令 ， 实现了位置闭环PD控制]



## /BasicWBR 

[20240312]时间过得很快



## /BasicLQR



# 关于常用的一些内容

（在各个文件当中不断习得的，可能会反复用到的内容）

## 各种需要的前置包

```
dynamic_reconfigure gazebo_plugins gazebo_ros gazebo_ros_control roscpp rospy std_msgs urdf xacro
```



## tasks.json 文件

```json
{
// 有关 tasks.json 格式的文档，请参见
    // https://go.microsoft.com/fwlink/?LinkId=733558
    "version": "2.0.0",
    "tasks": [
        {
            "label": "catkin_make:debug", //代表提示的描述性信息
            "type": "shell",  //可以选择shell或者process,如果是shell代码是在shell里面运行一个命令，如果是process代表作为一个进程来运行
            "command": "catkin_make",//这个是我们需要运行的命令
            "args": [],//如果需要在命令后面加一些后缀，可以写在这里，比如-DCATKIN_WHITELIST_PACKAGES=“pac1;pac2”
            "group": {"kind":"build","isDefault":true},
            "presentation": {
                "reveal": "always"//可选always或者silence，代表是否输出信息
            },
            "problemMatcher": "$msCompile"
        }
    ]
}

```









## Gazebo

### Gazebo&Launch

#### 模型位置

​	设置加载到Gazebo当中的模型的初始位置。

```xml
<node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description"  />

<!-- 
    在 Gazebo 中加载一个机器人模型，该功能由 gazebo_ros 下的 spawn_model 提供:
    -urdf 加载的是 urdf 文件
    -model mycar 模型名称是 mycar
    -param robot_description 从参数 robot_description 中载入模型
    -x 模型载入的 x 坐标
    -y 模型载入的 y 坐标
    -z 模型载入的 z 坐标
-->

```

