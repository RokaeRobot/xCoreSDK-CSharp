# 预编译库 / Prebuilt Libraries

本目录下的 `xCoreSDK_cli.dll`（及可选的 `.pdb`）不在 Git 仓库中托管，请从 GitHub Releases 下载：

https://github.com/RokaeRobot/xCoreSDK-CSharp/releases/tag/v0.5.1

下载 `xCoreSDK-CSharp-0.5.1-win.zip`，在**仓库根目录**解压。

Expected files:

```
xCoreSDK-CSharp/xcore.lib/xCoreSDK_cli.dll
xCoreSDK-CSharp/xcore.lib/xCoreSDK_cli.xml   # 已在 Git 中，Release 包亦含副本
```

`xCoreSDK_cli.xml` 为 API 文档，已随源码托管，便于 IDE 智能提示；运行仍需 `.dll`。
