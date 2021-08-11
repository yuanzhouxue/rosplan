# ROSPlan + TIAGo

## Info

此工作空间集成了rosplan与tiago的软件包，可将两者结合起来使用。

另外，工作空间中还有以下几个包：
 - rosplane: ROSPlan Extension, 对rosplan的扩展，包含抓取和观察两种action interface以及动作失败重规划的监控节点（Monitor Node）
 - test: 主要包含一些脚本、pddl和launch文件
 - xyz_nav: 对ROS中的navfn::NavfnROS的复制，它是一种全局规划器（Global Planner）

## 如何使用此工作空间

先克隆仓库

```bash
git clone git@gitee.com:yuanzhouxue/rosplan_ws.git
cd rosplan_ws
```

然后使用脚本初始化子模块（包括tiago,rosplan和rosplan_dependencies）

```bash
./init.bash
```

安装一些ROS依赖包

```bash
./install_dependencies.bash
```

编译工作空间

```bash
catkin build
```

