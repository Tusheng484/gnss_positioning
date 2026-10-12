# 配置与运行说明

通过 `--config=文件路径` 指定配置。可复制 [离线示例](../config/params.example.yaml) 为 `config/params.yaml`；已有配置不应直接覆盖。相对路径基于进程工作目录，而非配置文件目录。

## 输入与输出

| 参数 | 说明 |
| --- | --- |
| `io.input_mode` | `file` 为二进制文件回放；`network` 为 TCP 输入 |
| `io.input_path` | OEM 二进制日志位置 |
| `io.net_ip` / `io.net_port` | TCP 服务 IPv4 地址 / 端口；文件模式下不连接 |
| `io.output_dir` | 结果目录，同名输出会覆盖 |
| `output.terminal_output` | 历元摘要、配置回显及最终统计 |
| `output.file_output` | 总落盘开关；关闭后不写结果或 TCP 原始流 |
| `output.file_spp_solution` | 输出定位结果 |
| `output.file_sat_states` | 输出卫星状态 |
| `output.file_ephemeris` | 输出 GPS / BDS 星历 |
| `output.file_observations` | 输出解码观测 |
| `runtime.enable_spp` | 是否进行定位 |
| `runtime.enable_sat_state` | 是否计算独立输出的卫星状态，不替代 SPP 内部轨道计算 |
| `runtime.systems.gps` / `bds` | 参与定位及卫星状态输出的系统，至少开启一个 |

TCP 示例（替换为自己有权限访问、输出 OEM 二进制流的服务）：

```yaml
io:
  input_mode: network
  input_path: data/NovatelOEM20211114-01.log
  net_ip: 127.0.0.1
  net_port: 7003
  output_dir: output/network
```

仅替换完整配置的 `io` 段，其余段保留。接口不支持域名、NTRIP 握手或鉴权配置。接收超时后继续等待，服务关闭连接后结束，未实现自动重连。当前接收超时为 10 秒，它不是连接建立超时。落盘开启时额外写入 `realtime_input.log`。

## 定位策略

| 参数 | 示例值 / 范围 | 说明 |
| --- | --- | --- |
| `spp.frequency_mode` | `DUAL_FREQ_IF` | 双频无电离层组合 |
| `spp.frequency_mode` | `SINGLE_FREQ_KLOBUCHAR` | 单频伪距，模型是否应用还受电离层开关控制 |
| `spp.enable_tropo` | `true` | 主流程调用 Saastamoinen 对流层模型 |
| `spp.enable_tgd` | `true` | 相应频率组合下的群延迟改正 |
| `spp.enable_klobuchar_model` | `false` | 单频下是否启用 Klobuchar；IF 模式必须关闭 |
| `spp.elev_mask_deg` | 10，范围 0–90 | 高度角掩码，度 |
| `spp.snr_mask_dbhz` | 30，范围 0–70 | 信噪比掩码，dB-Hz |
| `spp.gps_klobuchar.enabled` / `bds_klobuchar.enabled` | `false` | 是否提供对应系统参数 |
| `alpha` / `beta` | 四元素数组 | 按模型广播参数约定填写；禁用时必须为零 |

IF 模式要求两个系统的 Klobuchar 参数均关闭。单频启用模型时，需要开启全局模型开关及相应系统参数开关，填写与观测时段相符的广播系数，不要把零数组当成课程数据的实测系数。

仅 GPS、单频但不使用电离层模型时，设置 GPS=true、BDS=false，频率为 `SINGLE_FREQ_KLOBUCHAR`，`enable_klobuchar_model=false`，两个系统参数开关为 false、数组为零。启用模型时按 `Config::validate()` 的系统与模型依赖规则配置。

## 参考坐标与统计

`reference_ecef.{x,y,z}` 为参考 ECEF 坐标，米。非零值用于 REF-ECEF 和 ENU；全零时以首个成功定位解为参考。后者只能分析相对位移或稳定性，不能作为绝对定位准确度。

输出可能包含失败或未收敛历元，文本没有独立 `ok` 列。统计前应结合终端摘要与有效性筛选；早期缺星历、零坐标、非有限值或异常 DOP 会污染统计。不能只以输出行数作为成功历元数。

## 解析器限制与故障定位

自带轻量解析器支持示例中的缩进键值、布尔值、数值及四元素数组，并非完整 YAML 实现。使用空格缩进，不使用锚点、多行字符串等复杂类型；路径避免 `#`。未知字段会报错。

| 现象 | 检查项 |
| --- | --- |
| 找不到 `Eigen/Dense` | 检查 `third_party/eigen/Eigen/Dense` 及目录嵌套 |
| 缺 GCC 运行时 DLL | 将编译所用 UCRT64 `bin` 加入运行时 PATH |
| 配置打不开 | 使用 `--config=...` 并检查启动目录 |
| 输入打不开 | 检查日志路径；RINEX 不能替代 OEM 日志 |
| `SPP_OK=0` | 检查消息、星历、双频观测、系统开关和筛选阈值 |
| 误差很大 | 检查参考坐标、时间及系统匹配，剔除失败历元 |

退出码：`0` 为流程结束，`1` 为输入 / 输出初始化或读取失败，`2` 为主函数捕获的配置或其他标准异常。服务正常断开也可能返回 0，仍需检查统计。
