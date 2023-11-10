# xCore SDK 机器人控制接口 C#版本

## 兼容性

### 机器人控制器
* xCore >= v2.1.0
### 编译
* .NET Framework >= 5.0
* 支持/clr编译选项，不支持/clr:netcore

## 工程说明及使用方法

### 文件说明
* `xCoreSDK_CSharp\xcore.lib` xCore-SDK DLL, XML文档
* `xMateRobotDemo.cs` `PCB4RobotDemo.cs`包括了接口调用方法的示例
### 使用方法
* 使用VS2017或更高版本打开`xCoreSDK_CSharp.sln`

#### 硬件设置
xCore SDK通过以太网（TCP/IP）连接机器人。C#只包含非实时接口，对网络的要求并不高，通过有线或无线连接皆可，使工作站PC和机器人连接同一局域网。
#### 机器人设置
* xCore SDK在使用前不需要通过Robot Assist打开相关功能。
* xCore SDK为需要授权的功能，如遇“功能未授权”的错误信息，请联系客户支持人员