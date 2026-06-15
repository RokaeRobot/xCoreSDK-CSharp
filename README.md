# xCore SDK 机器人控制接口 C# 版本

xCore SDK 编程接口库是珞石机器人提供给客户用于二次开发的软件产品（C# / CLR 封装）。

## 兼容性

### 机器人控制器

* xCore >= v3.2.1

### 运行环境

* .NET >= 5.0（推荐 .NET 7）
* Windows x64
* 支持 `/clr` 编译选项，不支持 `/clr:netcore`

## 获取预编译库

本仓库包含示例、WinForms 工程及 API 文档（`xCoreSDK_cli.xml`），**不包含** `xCoreSDK_cli.dll` / `.pdb`。这些文件通过 [GitHub Releases](https://github.com/RokaeRobot/xCoreSDK-CSharp/releases) 分发。

### 获取步骤

1. **克隆本仓库**
   ```bash
   git clone https://github.com/RokaeRobot/xCoreSDK-CSharp.git
   cd xCoreSDK-CSharp
   ```
2. **确认 SDK 版本** — 须与 Release 版本一致（当前为 **v0.5.1**，见 [CHANGELOG.md](CHANGELOG.md)）。
3. **打开对应 Release 页面** — [Release v0.5.1](https://github.com/RokaeRobot/xCoreSDK-CSharp/releases/tag/v0.5.1)  
   链接格式：`https://github.com/RokaeRobot/xCoreSDK-CSharp/releases/tag/v{VERSION}`
4. **下载** `xCoreSDK-CSharp-{version}-win.zip`。
5. **在仓库根目录解压**，使文件落入 `xCoreSDK-CSharp/xcore.lib/`。

### 解压后目录结构

```
xCoreSDK-CSharp/xcore.lib/
  xCoreSDK_cli.dll
  xCoreSDK_cli.xml
  xCoreSDK_cli.pdb          # 若 Release 包中含调试符号
```

## 工程说明及使用方法

### 文件说明

* `xCoreSDK-CSharp/xcore.lib/` — SDK DLL 与 XML 文档（DLL 从 Release 获取）
* `MainForm` — 基础功能接口示例
* `ApplicationForm` — 其它示例（含机器人状态数据读取）
* `examples/` — 各接口调用示例
* `share/doc_html/` — API HTML 文档

### 使用方法

* 使用 Visual Studio 2017 或更高版本打开 `xCoreSDK-CSharp.sln`
* 编译平台选择 **x64**
* 运行应用程序：选择机型类别，填入机器人 IP，选填本机地址（实时状态读取时需要），点击「连接」

#### 说明

* 示例中未适配 PCB3 轴与 CR5 轴机型，请参考其它机型的接口调用
* WinForm 界面仅包含部分功能，更多示例见 `examples/`

### 硬件设置

xCore SDK 通过以太网（TCP/IP）连接机器人。C# 版本仅含非实时接口，有线或无线均可，使 PC 与机器人处于同一局域网。

### 机器人设置

* xCore SDK 使用前无需在 Robot Assist 中单独打开相关功能
* 遇「功能未授权」请联系珞石客户支持

## License

> Copyright (C) 2026 ROKAE (Beijing) Technology Co., LTD.
