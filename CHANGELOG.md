# CHANGELOG

## v0.7.1 2026-03-03

* 修复
  * 实时模式关节点位跟随功能优化，处理奇异点问题
  * 设置工具工件信息(setToolInfo, setWobjInfo) 时旋转角度和负载信息设置错误

## v0.7.0 2026-01-16

* 兼容性
  * xCore >= v3.2.0
* 新增
  * ServoJ功能: setServoJoint(), stopServoJoint(), sendCommand()
  * 导出控制器备份exportBackup() 和升级 upgrade() 接口
  * 力控指令: 阻抗力限幅, 阻抗速度限幅, 设置关节力控带宽、摩擦力补偿系数
  * 计算全部逆解 calcAllIkSolutions()
  * 关闭工控机 shutdownSystem()
  * 查询控制器日志增加偏移选项
  * 读取DH参数 getRobotCfg_DHparam()
* 优化
  * 隔离内部引入的spdlog, 避免冲突
  * 实时模式运动延迟问题，增大超时时间上限到20ms
* 修复
  * 末端工具指令兼容新旧固件

## v0.5.1 2025-07-04

* 分发
  * 预编译库（`xCoreSDK_cli.dll`）改由 [GitHub Releases](https://github.com/RokaeRobot/xCoreSDK-CSharp/releases) 提供，不再使用 Git LFS
* 兼容性
  * xCore >= v3.0.2
* 新增
  * 同时获取多个数据getStateList()
  * TCP连接断开增加日志，日志路径为执行目录下logs；正逆解计算增加日志
* 修复
  * 网络连接断开后，运动事件监听无法恢复的问题，通过connectToRobot或setEventWatcher可再次设置监听
  * 查询工程工具工件崩溃问题
  * 版本号向下兼容
  * moveReset重置指令计数，避免由于网络不稳定引起的计数错误问题

## v0.5.0 2024-12-30

* 兼容性
  * xCore >= v3.0.1
* 新增
  * 支持控制导轨
    * 带导轨的联动运动；
    * 读写导轨参数 setRailParameter(), getRailParameter()
    * 导轨Jog
  * 笛卡尔空间运动指令可达性校验 checkPath()
  * 写寄存器数组
  * XMS, XMC机型末端485通信相关 setxPanelRS485(), XPRWModbusRTUReg(), XPRWModbusRTUCoil()
  * 运动停留指令MoveWait，运动指令自定义信息
  * RL工程执行状态事件通知
  * 机器人实时状态数据读取（工业3,4轴机型不支持）
  * 给定工具工件坐标系下的正逆解计算
  * 无末端按键拖动
* 修复
  * 工业六轴机型本机地址设置问题
  * 进入协作模式后读取的机器人状态错误问题

## v0.4.1.a 2024-08-16

* 兼容性
  * xCore >= v2.2.1, 部分新增特性需要xCore >= v2.2.2
* 新增
  * 运动指令速度细化，增加关节速度百分比jointSpeed 和空间旋转速度rotSpeed (需要xCore版本v2.2.2)
  * 急停复位接口recoverState()
  * SDK日志可通过本地文件配置
* 修复 (需要xCore版本v2.2.2)
  * Jog步长在示教器显示的问题
  * 运动缓存错误码没有传出的问题
  * 碰撞检测负载设置问题

## v0.4.0 2024-04-30

* 兼容性
  * xCore >= v2.2
* 新增
  * 支持CR5轴机型
  * 所有非实时力控指令
  * 力矩传感器标定calibrateForceSensor()
  * SDK执行日志
  * 加速度读写接口getAcceleration(), adjustAcceleration()
  * 末端按键读取 getKeypadState()
  * 设置基坐标系 setBaseFrame()
  * 打开关闭三种奇异规避方式
  * 其它新增: MoveSP指令增加偏移项; 全圆指令增加旋转类型, 等
* 修复
  * 非实时接口多线程阻塞问题
  * 其它已知问题
* 变更
  * 使用协作CR和SR机型，并且程序中MoveJCommand用到了confData，需要在程序中加 setDefaultConfOpt(true), 让confData生效;
  * 错误码和异常信息语言根据用户PC系统语言设置而定，中文返回中文信息，非中文返回英文信息;
  * 拖动回放replayPath()，由调用完立即开始运动，改为需要moveStart()才开始运动，并且可以和其它运动指令一起下发;
  * 通过setToolset()函数设置的工具工件组，优化为右上角的工具工件显示"toolx", "wobjx"，并且状态监控里看到的位姿会同步更新。

## v0.3.4 2023-11-16

* 兼容性
  * xCore >= 2.1
* 新增
  * 第一个正式版本，增加运动、通信、RL工程等非实时接口若干
