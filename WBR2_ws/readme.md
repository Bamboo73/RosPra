# WBR2_ws 工程文件说明

---

这是一个工程log

## Log(过程)

### 240509

在功能包wbr_pkg 下面的include 目录中创建 wbr_pkg 目录，然后再在此子目录下创建.h文件。

此时，CPP文件中调用这些文件的方式为：

```cpp
#include "wbr_pkg/xxx.h"
```

注意：此引号所对应的，就是功能包下面的inlcude 目录，因为此目录下有一个子目录wbr_pkg，所以引用的时候有这样的一个前缀。

另外，还需要修改功能包的CMakeList.txt文件：

```cpp
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

将include_directories中的include项目解除注释。



### 240509

计划在构建工程文件目录的时候，学习一下CPP面相对象变成的方式。

感觉可以切换到双系统取处理问题了

## Notes(整理)

