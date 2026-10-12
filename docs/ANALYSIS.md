# 历史结果分析脚本

分析依赖不影响 C++ 构建。优先用项目已有 Python 环境；新环境可执行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

`requirements.txt` 声明使用到的库，未锁版本，不是完整环境复刻文件。建议 Python 3.9 或更新版本；实际验证范围见 [验证记录](VALIDATION.md)。

## 参考解对比：analyze.py

按历史定义读取 `spp_solution.txt` 前 23 个字段，以内嵌参考坐标计算 ECEF 差及三维距离，并按 GPS 周与周内秒和参考解对比其他字段。实际输出只有前 22 列为固定数值布局，第 23 字段开始是卫星编号列表；脚本中的 `n` 转为数值后为 NaN，不应据此计数，参与卫星数应使用 `GS + BS`。

运行前修改 `main()` 中的两个本机绝对路径，指向课程参考 `.pos` 和自己的 `spp_solution.txt`：

```powershell
.\.venv\Scripts\python.exe analyze.py
```

参考文件必须与脚本列布局兼容，任意外部 `.pos` 格式不一定适用。`Total matched epochs` 实际为满足坐标非空条件的结果行数，不等于与参考文件的时间匹配数，也不代表成功定位历元数。脚本未自动剔除零坐标等失败记录，统计前需筛选有效数据。

本次离线回放的前 8 个失败历元仅有 22 个字段，因此被这个脚本的长度检查跳过，最终读入 2,673 行；这属于该数据的特定格式表现，不能替代通用的成功状态筛选。

## 精度与稳定性：report_spp_precision_analysis.py

读取前 22 列，生成卫星数量、ENU / 三维误差、速度稳定性、几何相关性图表和摘要。先修改文件顶部 `DEFAULT_INPUT_FILE` 和 `DEFAULT_OUTPUT_DIR`：

```powershell
.\.venv\Scripts\python.exe report_spp_precision_analysis.py
```

脚本无命令行参数接口，保留课程报告绘图设置；中文需要对应字体，图像按 600 dpi 导出。`seaborn` 对此脚本是可选的，依赖清单仍列出以便重现默认样式。

读取函数只要求前 22 列，因此同一份回放结果读入 2,681 行，其中 8 行位置为 NaN；`main()` 会继续剔除 ENU 缺失行。它和 `analyze.py` 的读取行数差异由解析门槛导致，并非丢失了成功解，其他异常记录仍需按实际数据筛选。

脚本直接用 ENU 计算误差，假定参考点是真值；使用首个成功解为参考时应解释为相对稳定性。动态接收机的速度模长不能当作测速误差，需要独立速度参考。

图表时间由 GPS 周和周内秒相加得到，未减闰秒，不能直接标为 UTC。批量绘图前先筛选有效历元并检查输入大小。

## 未完整归档的实验

本地 `report_batch_config_comparison.py` 和 `config/comparison_manifest.json` 依赖的 `plot/plot.py` 当前缺失，保留本地、排除本次提交，未宣称可运行。完整报告和提交压缩包也保留本地。
