这是一份学习如何使用ros control 的文档

## 已写完

## 前言

​	不得不说关于ros_control的内容还有很多需要了解和学习的，至今还是有很多内容没有get到。

​	这份文档的目的在于：

​	能够基本的解决在gazebo中机器人的位置控制、力控制。 

.vscode/tasks.json file

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







```
dynamic_reconfigure gazebo_plugins gazebo_ros gazebo_ros_control roscpp rospy std_msgs urdf xacro
```

​	假设xacro文件（urdf文件）没有什么问题。



​	这里只写关节控制器，不写传感器的内容。



## effort control

最基础的控制方式，

需要做的事情如下：

### 1、 transmission

### 写 transmit ，将机器人的关节与力矩控制界面绑定

```xml
    <xacro:macro name="transmission_macro" params = "joint_name">
        <transmission name="${joint_name}_transmission">
            <type>transmission_interface/SimpleTransmission</type>

            <joint name="${joint_name}">
                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                <!-- 在 Gazebo      当中用 ： EffortJointInterface -->
                <!-- 在 RobotHW     当中用 ： hardware_interface/EffortJointInterface -->
            </joint>

            <actuator name="${joint_name}_motor">
                <mechanicalReduction>1</mechanicalReduction>
                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
                <!-- 在 Indigo 之前的版本，hardwareInterface只在此处定义（和我noetic什么关系x） -->
            </actuator>
        </transmission>
    </xacro:macro>
```

这是一段关键的xacro宏。对于力矩控制，要写成hardware_interface/EffortJointInterface。

然后记得把插件打开：

```xml
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/robot_simple</robotNamespace>
        </plugin>
    </gazebo>
```

AT（重要！）：这里的namespace 以及 yaml文件的namespace ，以及launch 文件当中控制器生成的namespace，要统一。

### 2、 yaml

使用yaml文件定义力矩控制器，方式如下：

```yaml
robot_simple:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  my_effort_controller:
    type: effort_controllers/JointEffortController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

effort_controllers/JointEffortController

### 3、launch 文件

```xml
    <!-- 1 Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find simplebotpkg)/yaml/cfg.yaml" command="load"/>
    <!-- 2 load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="/robot_simple" args="
        joint_state_controller 
        my_effort_controller"
        />
```

找到工程文件中的yaml文件，加载。

启动controller_spawner ，生成控制器。

下面要的就是在yaml中定义过的控制器。

### 4、写自己的控制器

这之后的工作就不难了，将自己需要的控制器写好就行,比如：

```
CmdPub  = nt.advertise<std_msgs::Float64>("/robot_simple/my_effort_controller/command",10);
```







## Pos control

位置控制的内容会相对多一些，但大体上一样。

### 1、 transmission

### 写 transmit ，将机器人的关节与力矩控制界面绑定

```xml
    <xacro:macro name="transmission_macro" params = "joint_name">
        <transmission name="${joint_name}_transmission">
            <type>transmission_interface/SimpleTransmission</type>

            <joint name="${joint_name}">
                <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
                <!-- 在 Gazebo      当中用 ： EffortJointInterface -->
                <!-- 在 RobotHW     当中用 ： hardware_interface/EffortJointInterface -->
            </joint>

            <actuator name="${joint_name}_motor">
                <mechanicalReduction>1</mechanicalReduction>
                <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
                <!-- 在 Indigo 之前的版本，hardwareInterface只在此处定义（和我noetic什么关系x） -->
            </actuator>
        </transmission>
    </xacro:macro>

    <xacro:transmission_macro joint_name="joint1" />

    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/robot_simple</robotNamespace>
        </plugin>
    </gazebo>
```

这是一段关键的xacro宏。

位置控制使用的Interface是hardware_interface/PositionJointInterface

【AT：我看到似乎有照样用Effort的，不过对应的yaml文件有所不同。这里就先用这个吧】

### 2、 yaml

使用yaml文件定义力矩控制器，方式如下：

```yaml
robot_simple:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  my_pos_controller:
    type:  position_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
    #似乎，pos控制没有pid参数...

  gazebo_ros_control:
    pid_gains:
      Left_hip_joint:
        p: 100.0
        i: 0.01
        d: 10.0
      Left_knee_joint:
```

首先，这里的控制器类型是position_controllers/JointPositionController。

注意，这里要多加一个gazebo_ros_control 的标签。不然会报错没有p增益：

```cmd
No p gain specified for pid.  Namespace: /robot_simple/gazebo_ros_control/pid_gains/joint1
```

注意2： gazebo_ros_control和工作空间 robot_simple 要有缩进关系，不能平行。

### 3、launch 文件

```xml
<!-- 以下部分是新加入的关于控制相关的内容 -->
    <!-- 1 Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find simplebotpkg)/yaml/cfg_pos.yaml" command="load"/>
    <!-- 2 load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
        output="screen" ns="/robot_simple" args="
        joint_state_controller 
        my_pos_controller"
        />

```

找到工程文件中的yaml文件，加载。

启动controller_spawner ，生成控制器。

下面要的就是在yaml中定义过的控制器。

### 4、写自己的控制器

这之后的工作就不难了，将自己需要的控制器写好就行，比如：

```
CmdPub  = nt.advertise<std_msgs::Float64>("/robot_simple/my_pos_controller/command",10);
```









