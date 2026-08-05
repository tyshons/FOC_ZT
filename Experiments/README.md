# FOC_ZT 实物实验目录

> 转台融合版本请先阅读
> `Docs/TURNTABLE_MERGE_GUIDE.md`。本版本由转台下位机统一完成初始化，
> 实验模式通过 `Experiment_Control` 管理。

本目录集中保存所有实物实验专属的控制、配置、测试和说明文件。驱动层仍保留在原来的 `Core`、`Drivers` 和 `Lib` 目录中。

## 目录

```text
Experiments/
├─ Inc/
│  ├─ Experiment_Control.h
│  ├─ Experiment_Config.h
│  ├─ Experiment_Baseline.h
│  ├─ Experiment_ESO.h
│  ├─ Experiment_FixedFeedforward.h
│  ├─ Experiment_FixedFF_Sweep.h
│  ├─ Experiment_LearningFeedforward.h
├─ Src/
│  ├─ Experiment_Control.c
│  ├─ Experiment_Baseline.c
│  ├─ Experiment_ESO.c
│  ├─ Experiment_FixedFeedforward.c
│  ├─ Experiment_FixedFF_Sweep.c
│  └─ Experiment_LearningFeedforward.c
├─ Tests/
│  ├─ experiment_control_host_test.c
│  ├─ fixed_ff_sweep_host_test.c
│  ├─ foc_math_host_test.c
│  ├─ merged_runtime_mode_host_test.c
│  ├─ pid_control_host_test.c
│  └─ pos_process_host_test.c
└─ Docs/
   ├─ TURNTABLE_MERGE_GUIDE.md
   ├─ PHYSICAL_EXPERIMENT_GUIDE.md
   ├─ FIXED_FF_SWEEP_GUIDE.md
   └─ FIRMWARE_CHANGELOG_AND_PLANT_MAPPING.md
```

## 主程序接口

转台融合版本在 `main.c` 初始化区调用：

```c
Experiment_Control_Init();
```

主循环调用：

```c
Experiment_Control_Background();
```

`Lib/Src/FOC_Control.c` 的速度环调用
`Experiment_Control_Update()`。旧的重复实验入口已经移除。

## 实验模式

通过调试器修改 `g_experiment_mode_request`：

| 值 | 模式 | 独立算法文件 |
| ---: | --- | --- |
| `0` | 三闭环基线 | `Experiment_Baseline.c` |
| `1` | 三闭环 + ESO | `Experiment_ESO.c` |
| `2` | 三闭环 + 固定前馈 | `Experiment_FixedFeedforward.c` |
| `3` | 三闭环 + ESO + 固定前馈 | ESO 与固定前馈文件 |
| `4` | 三闭环 + ESO + 学习前馈 | `Experiment_LearningFeedforward.c` |
| `5` | ESO + 固定前馈 + 学习前馈完整组合 | 三个算法文件 |
| `6` | 三闭环基线 + 固定前馈主动扫阶 | `Experiment_FixedFF_Sweep.c` |

上电默认模式为 `0`。模式切换时会复位 ESO 并丢弃当前未完成的学习圈，不会清除已经完成的学习表。

模式 `6` 使用机械角同步的正、负小电流注入辨识 1–40 阶速度响应，并给出固定前馈余弦/正弦电流建议值。结果只保存在调试变量中，不会自动写入或启用 `Experiment_FixedFeedforward.c`。具体操作见 `Docs/FIXED_FF_SWEEP_GUIDE.md`。

学习相关请求：

- `g_learning_update_request`：`1` 表示更新学习表，`0` 表示冻结。
- `g_learning_reset_request`：写 `1` 请求清空学习表，后台处理后自动恢复为 `0`。
- `g_learning_mode_request`：`0` 普通、`1` 逐阶选择性、`2` 全局相关。

## 分层规则

- `Core`、`Drivers`、SSI 等驱动文件只负责外设和采样，不放实验选择逻辑。
- `Lib/Src/FOC_Control.c` 只在速度环调用 `Experiment_Control_Update()`，不包含具体实验算法。
- `Experiment_Control.c` 只调度实验模式和组合电流分量。
- 每个具体算法保存在独立文件，新增实验时不要把实现写进 `main.c`。
