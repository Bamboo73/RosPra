# BasicGz_ws 工程文件

---

记录基本的构建urdf（xacro）与gazebo整合的实现方法。

尝试最好不要记录与[教程](http://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/66-urdfji-cheng-gazebo.html)重复的东西。

【240229 初稿】



## 功能包

编写代码，依赖的包是 roscpp rospy std_msgs;

建立模型，依赖的包是 urdf xacro

进行仿真，依赖的包是 gazebo_ros  gazebo_ros_control gazebo_plugins





### robot1 相关

robot1 包含：robot1.urdf 与 robot1.launch。

功能：实现基本的urdf在gazebo当中的载入。

#### 【AT1：urdf内容】

​	urdf与gazebo联调的时候，必须有碰撞、惯性属性。毕竟是“仿真实际环境”，必要的互动参数还是需要有的。

#### 【AT2：launch文件内容】

```xml
<launch>
    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" textfile="$(find basic_gz_pkg)/urdf/robot1.urdf" />
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />
    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model robot111 -param robot_description"  />
</launch>
```

第二行的作用为生成一个空的gazebo世界。对我现在而言这个够了。之后加东西再写。

第三行：

```xml
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf 
    -model robot111 
    -param robot_description
    -x 0
    -y 0
    -z 1"  />
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

### robot2(/xacro2)相关

​	就是实现一下xacro和gazebo的联动，主要是复制粘贴，足矣。

​	robot2.launch  当中有关于如何加载一个包含环境的gazebo的世界的加载代码。





