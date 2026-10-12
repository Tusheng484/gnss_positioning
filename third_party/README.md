# 第三方依赖

主程序使用 Eigen 进行矩阵计算。CMake 当前读取 `third_party/eigen/`，没有使用系统级 `find_package(Eigen3)`，也不会自动下载依赖。

在仓库根目录执行：

```powershell
git clone --depth 1 --branch 3.4.1 https://gitlab.com/libeigen/eigen.git third_party/eigen
Test-Path third_party/eigen/Eigen/Dense
```

最后一条命令应返回 `True`。也可从 [Eigen 官方仓库](https://gitlab.com/libeigen/eigen) 获取相同版本源码并解压，确保没有多嵌套一层。若本机已有该目录，无须再次克隆或覆盖。

Eigen 为仅头文件依赖，无须单独编译。本次本地构建使用头文件版本 3.4.1；其源码及许可随官方发布物提供，未复制进本仓库。`.gitignore` 排除依赖目录。

TCP 输入依赖 Windows 自带的 Winsock 2，CMake 在 Windows 上链接 `ws2_32`。当前 `SocketStream.cpp` 直接包含 Windows 头文件，不能据 CMake 平台条件判断为已支持 Linux / macOS。
