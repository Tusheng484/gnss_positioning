# GNSS Positioning · GPS / BDS 单点定位

基于 **C++17、CMake 和 Eigen** 实现的 GNSS 标准单点定位（SPP）课程作业。从 NovAtel OEM 二进制数据中解析观测值与广播星历，计算卫星位置、速度及钟差，完成 GPS / BDS 伪距定位和多普勒测速。

本仓库用于归档历史课程实践、展示算法实现与程序组织。当前实现面向 **Windows**；原始观测日志、完整课程报告和本机运行配置单独保存在本地。

## 功能

| 模块 | 已实现内容 |
| --- | --- |
| 数据解码 | OEM 帧同步、CRC32 校验；RANGE（43）、GPSEPHEM（7）、BDSEPHEMERIS（1696） |
| 导航系统 | GPS L1/L2、BDS B1I/B3I；GPS、BDS 或联合解算 |
| 卫星状态 | 广播星历轨道计算、ECEF 位置与速度、卫星钟差与钟速 |
| 定位解算 | 单频伪距 / 双频无电离层组合、高度角加权最小二乘、系统间偏差估计 |
| 误差处理 | 地球自转、卫星钟差、相对论效应、TGD、Saastamoinen 对流层、可配置 Klobuchar 电离层改正 |
| 质量控制 | 信噪比和高度角筛选、GF 历元间跳变检查、多普勒一致性检查、残差剔除 |
| 输入 / 输出 | 文件回放、Windows TCP 实时字节流；定位、观测、星历与卫星状态文本输出 |

不包含 RTK、PPP 或载波相位模糊度解算；当前主程序不直接读取 RINEX、SP3 或 `.pos` 文件。TCP 模式接收原始 OEM 二进制流，不包含 NTRIP 协议或鉴权功能。

## 算法与模块

```mermaid
flowchart LR
    A[OEM 日志 / TCP 字节流] --> B[帧同步与 CRC 校验]
    B --> C[观测值]
    B --> D[广播星历缓存]
    C --> E[观测筛选与频率组合]
    D --> F[发射时刻与卫星状态]
    E --> F
    F --> G[误差改正与伪距方程]
    G --> H[加权最小二乘与残差剔除]
    H --> I[位置 / 钟差 / DOP]
    H --> J[多普勒测速]
```

详细模块接口、公式和流程见 [项目架构与算法流程](项目架构与算法流程.md)。

## 快速开始

### 1. 准备环境与依赖

- Windows，支持 C++17 的编译器。
- CMake ≥ 3.15、Git。
- Eigen 3.4.1，放到 `third_party/eigen/`，见 [依赖说明](third_party/README.md)。
- Python 仅用于可选结果分析，不参与 C++ 程序构建。

以下命令在 **PowerShell、仓库根目录** 执行。先克隆仓库，再获取 Eigen；私有仓库需要对应访问权限。

```powershell
git clone https://github.com/Tusheng484/gnss_positioning.git
Set-Location gnss_positioning
git clone --depth 1 --branch 3.4.1 https://gitlab.com/libeigen/eigen.git third_party/eigen
```

### 2. 构建

使用 MSYS2 UCRT64 的 GCC 工具链。在 PowerShell 中将安装路径替换为自己的位置，保证 `g++`、`mingw32-make` 及运行时 DLL 来自同一套工具链：

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
```

产物为 `build/bin/gnss_positioning.exe`。如果 `build/` 曾使用另一种编译器或生成器，请使用新构建目录（如 `build-release/`），避免混用缓存。

当前源码直接依赖 Winsock；Linux / macOS 尚未适配。MSVC 构建未在本次归档中验证。

### 3. 准备观测数据与配置

将含广播星历与 RANGE 观测的 OEM 二进制日志放入 `data/`，例如 `data/NovatelOEM20211114-01.log`。输入要求见 [数据说明](data/README.md)。

新克隆的仓库中执行：

```powershell
Copy-Item config/params.example.yaml config/params.yaml
```

已有自己的 `params.yaml` 时直接编辑，保留原配置。示例默认使用离线文件、GPS+BDS、双频 IF 组合。将 `io.input_path` 改为日志路径，将 `reference_ecef` 改为测站已知 ECEF 坐标（米）。相对路径均相对于**启动程序时的工作目录**。

### 4. 运行

```powershell
.\build\bin\gnss_positioning.exe --config=config/params.yaml
```

必须使用 `--config=路径` 形式；解析器不支持 `--config 路径`。未指定参数时，历史代码仍使用原作者机器的绝对配置路径，因此建议始终显式传入配置。

程序在文件结束后打印处理摘要，`SPP_OK` 为成功解算历元数。退出码为 0 只表示处理流程结束，不能单独证明定位成功。重新运行会覆盖同一输出目录的同名结果，比较实验应分别设置 `io.output_dir`。

完整参数、单频设置及 TCP 输入见 [配置说明](docs/CONFIGURATION.md)。

## 输出与分析

示例默认结果写入 `output/file/`：

| 文件 | 内容 |
| --- | --- |
| `spp_solution.txt` | GPS 周 / 周内秒、ECEF、参考 ECEF、ENU、经纬高、速度、PDOP、精度指标、参与卫星数 |
| `sat_states.txt` | 广播星历卫星状态；位置 km、速度 m/s、钟差 μs、钟速 μs/s |
| `gps_ephem.txt` / `bds_ephem.txt` | GPS / BDS 广播星历参数 |
| `range_obs.txt` | 伪距 m、相位 cycle、多普勒 Hz、信噪比 dB-Hz |
| `realtime_input.log` | 仅 TCP 模式下保存的接收原始字节流 |

`spp_solution.txt` 的 `VX/m`、`VY/m`、`VZ/m` 是历史表头，实际单位为 **m/s**。`SigmaP = sigma0 × PDOP` 是模型内部指标，不等同于相对真值的实测定位误差。`range_obs.txt` 保存解码观测，系统开关主要控制后续解算和卫星状态计算。

`GS`、`BS` 是参与解算的 GPS / BDS 卫星数；历史表头末尾的 `n` 实际对应可变长度卫星编号列表，不是一个数值计数列。前 22 列为固定数值布局，失败历元可能包含 `nan`，分析时应先筛选有效记录。

可选分析依赖安装在自己的虚拟环境或项目 Conda 环境中：

```powershell
python -m pip install -r requirements.txt
```

`analyze.py` 提供课程参考解对比；`report_spp_precision_analysis.py` 提供 ENU、速度和几何指标统计与绘图。两个脚本保留历史路径，使用前需修改输入 / 输出位置，见 [分析说明](docs/ANALYSIS.md)。本次构建、文件回放和限制记录见 [验证记录](docs/VALIDATION.md)。

## 目录

```text
gnss_positioning/
├── README.md
├── CMakeLists.txt
├── config/params.example.yaml    # 可移植离线配置
├── data/README.md                # 外部数据准备方式
├── include/gnss_positioning/     # 模块接口
├── src/                         # 解码、轨道、SPP、配置和输入实现
├── third_party/README.md         # Eigen 获取方式
├── analyze.py                   # 历史参考解对比脚本
├── report_spp_precision_analysis.py
├── requirements.txt             # 可选分析依赖
├── docs/                        # 配置、分析与验证说明
└── 项目架构与算法流程.md
```

本机配置、原始数据、构建、输出、完整报告与提交压缩包由 `.gitignore` 排除，本地文件保留。仓库历史中已有的参考数据仍保留；本次整理不重写历史。

## 课程归档与使用范围

作者：Jiaoyang Wu（吴骄阳）。这是历史课程作业实现，适合学习协议解码、广播星历与 SPP 算法。实际精度依赖数据质量、频率组合、卫星几何及参考坐标，本仓库不承诺固定精度。

当前尚无自动化单元测试，未验证实时服务的持续可用性。原始课程日志和完整报告未随本次归档上传，复现定位结果需要自行准备输入数据。项目尚未指定开源许可证；Eigen 遵循其自身许可，课程资料使用需遵循原提供方要求。
