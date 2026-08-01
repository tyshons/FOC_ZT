# 转台上位机通信改动说明

本文档记录本次为转台下位机接入上位机通信所做的代码修改、已完成功能、协议符合性，以及目前仍存在的问题。

## 1. 依据

参考协议文件：

`C:/Users/11527/Documents/xwechat_files/wxid_mjl1ze8bmidh22_18a8/temp/RWTemp/2026-07/65620ab229cd0c918c11fa06d9cf38fa/XN-JS05-2026-001-光电吊舱通信协议规范-V1.0-20260109.doc`

协议核心要求：

- 串口通信，默认 `115200bps, 8N1`。
- 帧头固定为 `A5 5A`。
- 帧尾固定为 `0D 0A`。
- 多字节数据采用小端序。
- 校验和为除帧头、帧尾以外的数据累加后取低 8 位。
- 工作模式：
  - `0x00`: 系统断能
  - `0x01`: 系统使能
  - `0x02`: 伺服模式
  - `0x03`: 跟踪模式
  - `0x04`: 扫频模式

## 2. 下位机改动

### 2.1 新增正式通信模块

新增文件：

- `Lib/Inc/turntable_comm.h`
- `Lib/Src/turntable_comm.c`

核心入口：

- `Turntable_Comm_Init()`：初始化串口协议接收状态机。
- `Turntable_Comm_Task()`：在主循环中处理已接收完整帧，并周期发送状态帧。
- `Turntable_Comm_UartRxCpltCallback()`：USART1 单字节中断接收回调。
- `Turntable_Comm_UartTxCpltCallback()`：USART1 发送完成回调。
- `Turntable_Comm_UartErrorCallback()`：USART1 错误恢复回调。

协议模块默认启用：

```c
#define TURNTABLE_COMM_ENABLE 1
```

位置：

- `Lib/Inc/turntable_comm.h`

### 2.2 接入 CMake

修改文件：

- `CMakeLists.txt`

新增编译文件：

```cmake
Lib/Src/turntable_comm.c
Lib/Inc/turntable_comm.h
```

旧 VOFA、UART 编码器和 BISS-C 文件已经移除，控制角度统一来自 SSI。

### 2.3 接入主程序

修改文件：

- `Core/Src/main.c`

新增头文件：

```c
#include "turntable_comm.h"
```

初始化区新增：

```c
Turntable_Comm_Init();
```

主循环新增：

```c
Turntable_Comm_Task();
```

### 2.4 UART 回调分发

修改文件：

- `Lib/Src/turntable_comm.c`

正式上位机协议只使用 USART1，三个 HAL UART 回调均直接转交给
`turntable_comm`；USART3 不再初始化。

涉及回调：

- `HAL_UART_TxCpltCallback`
- `HAL_UART_RxCpltCallback`
- `HAL_UART_ErrorCallback`

### 2.5 移除旧 VOFA 通道

VOFA 文本/CSV 调参协议和对应源文件已经移除，避免与正式二进制协议混用。

## 3. 上位机改动

修改文件：

- `C:/Users/11527/Desktop/转台/转台界面/turntable_control.py`

将串口波特率从 `9600` 改为 `115200`：

```python
baudrate=115200
```

原因：

- Word 协议默认是 `115200bps`。
- STM32 当前 `Core/Src/usart.c` 里 USART1 也是 `115200`。
- 上位机如果保持 `9600`，即使协议帧格式正确，也无法正常通信。

## 4. 已完成功能

### 4.1 帧接收与校验

已实现：

- 查找帧头 `A5 5A`。
- 按帧尾 `0D 0A` 判断完整帧。
- 校验和检查。
- 错误帧丢弃。
- 串口错误后自动恢复接收。

### 4.2 系统使能/断能

支持协议：

```text
A5 5A 00 axis checksum 0D 0A
A5 5A 01 axis checksum 0D 0A
```

实现行为：

- `0x00`: 调用 `Motor_Disable()`
- `0x01`: 调用 `Motor_Enable()`

当前没有区分方位/俯仰/横滚轴，因为固件里目前只有一套实际电机控制链路。

### 4.3 伺服模式角度设置

支持 Word 表格中的双轴角度帧：

```text
A5 5A 02 01 az_float el_float checksum 0D 0A
```

也兼容当前上位机手动控制按钮实际发送的单轴格式：

```text
A5 5A 02 01 axis float checksum 0D 0A
```

实现行为：

- 方位轴 `axis=0x00`：写入 `position_given_sp`，驱动当前真实 FOC 位置目标。
- 俯仰轴 `axis=0x01`：目前只写入软件变量并回传给上位机。

### 4.4 PID 参数设置

支持伺服模式：

```text
A5 5A 02 00 axis loop Kp Ki Kd checksum 0D 0A
```

支持跟踪模式：

```text
A5 5A 03 03 axis loop Kp Ki Kd checksum 0D 0A
```

当前 loop 映射：

- `0x00`: Q 轴电流环，映射到 `iq_pid_inst`
- `0x03`: D 轴电流环，映射到 `id_pid_inst`
- `0x01`: 速度环，映射到 `speed_pid_inst`
- `0x02`: 位置环，映射到 `position_pid_inst`

注意：固件当前只有一套真实控制轴，所以 `axis` 参数暂时不区分不同电机，只用于协议兼容。

新增内部扩展：PID 参数查询/回传。

由于 Word 协议没有定义“读取当前 PID 参数”的指令，本次增加扩展模式 `0x05`：

查询当前 PID：

```text
A5 5A 05 10 mode axis loop checksum 0D 0A
```

其中 `loop=0xFF` 表示一次查询该轴全部 PID 环。

下位机回传 PID：

```text
A5 5A 05 11 mode axis loop Kp Ki Kd checksum 0D 0A
```

上位机打开手动控制面板时，会先用本地缓存填入输入框，然后发送 PID 查询；收到 `05 11` 回传后，再用下位机当前参数刷新输入框。

### 4.5 功能开关

支持跟踪模式：

```text
A5 5A 03 feature on_off checksum 0D 0A
```

支持伺服模式：

```text
A5 5A 02 feature on_off checksum 0D 0A
```

其中：

- 跟踪模式 `feature=0x00/0x01/0x02`: 可见光/红外/测距
- 伺服模式 `feature=0x03/0x04/0x05`: 可见光/红外/测距

当前实现只保存开关状态，没有接具体 GPIO、继电器或外设控制。

### 4.6 扫频模式参数接收

支持协议：

```text
A5 5A 04 axis amplitude start_hz end_hz period_count step_count checksum 0D 0A
```

当前实现：

- 能解析并保存扫频参数。
- 暂未真正执行扫频控制，也未生成扫频回传数据。

### 4.7 状态回传

下位机每 50ms 回传一次当前角度，格式按当前上位机代码解析方式实现：

```text
A5 5A az_float el_float 0D 0A
```

其中 `az_float` 直接发送水平轴实际位置 `current_angle_sp`，不再做 `-180~180` 归一化处理。

上位机当前 `parse_servo_tracking_data()` 正是按 12 字节解析：

```text
A5 5A + 方位实际 float + 俯仰实际 float + 0D 0A
```

## 5. 是否完全按照协议要求修改

结论：上位机下发指令的接收与解析，核心部分基本按 Word 协议实现；但不是 100% 完整闭环协议实现。

符合协议的部分：

- 帧头 `A5 5A`
- 帧尾 `0D 0A`
- 小端 float
- 下发命令校验和
- 工作模式码 `00/01/02/03/04`
- 伺服 PID 设置
- 跟踪 PID 设置
- 伺服双轴角度设置
- 功能开关命令
- 扫频参数帧解析
- 串口参数改为 `115200 8N1`

为了兼容当前上位机代码做的扩展：

- 上位机手动按钮发送的是单轴角度帧：`02 01 axis float`。Word 表格 5 是双轴角度帧，没有 axis 字节；但协议后面的示例又提到了设置方位角带 axis。下位机因此同时兼容两种格式。

未完全按协议或协议未定义清楚的部分：

- 状态回传帧没有加校验和，因为当前上位机代码只解析 12 字节：`A5 5A + 两个 float + 0D 0A`。Word 协议核心帧结构要求校验位，但文档没有明确给出状态上报帧格式。这里优先兼容上位机现有解析逻辑。
- 系统使能/断能命令里虽然带 axis，但当前下位机没有按 axis 分别使能不同轴。
- 扫频模式目前只解析参数，没有实现扫频运动和扫频数据回传。
- 功能开关目前只保存状态，没有驱动实际外设。
- 跟踪模式 PID 中的“图像环”目前映射到位置环 PID，没有单独图像环控制器。

## 6. 当前仍存在的问题

### 6.1 只有一个真实控制轴

固件当前只有一套 FOC 控制变量：

- `current_angle_sp`
- `position_given_sp`
- `speed_given_sp`
- `position_pid_inst`
- `speed_pid_inst`
- `id_pid_inst`
- `iq_pid_inst`

所以目前只有方位轴真正接入电机控制。俯仰轴只是软件变量，能被上位机设置和显示，但不会驱动第二个电机。

后续如果有第二轴，需要新增第二套：

- 编码器角度
- 目标角度
- 速度计算
- PID 实例
- PWM/电流采样/FOC 控制链路

### 6.2 状态回传不是严格标准帧

当前回传为了适配上位机，未包含校验和。

如果后续要严格按协议统一，建议把上位机状态解析改成：

```text
A5 5A payload checksum 0D 0A
```

然后下位机同步增加校验和。

### 6.3 上位机代码自身协议有不一致

上位机有些地方按双轴角度发送，有些地方按单轴角度发送：

- 主界面目标角度：`02 01 az_float el_float`
- 手动上下左右：`02 01 axis float`

下位机已经兼容两者，但从长期维护看，建议上位机协议格式最终统一。

### 6.4 PID 的 axis 参数暂未真正区分轴

上位机能发送“方位轴/俯仰轴”的 PID，但下位机当前只有一套 PID，因此无论 axis 是多少，都会改同一套 PID。

### 6.5 旧扫频协议已移除

原来只保存参数、没有驱动控制输出的 `TT_MODE_SWEEP` 路径已经删除。
当前主动扫阶只通过实验扩展协议和 `Experiment_FixedFF_Sweep` 实现。

### 6.6 VOFA 已移除

正式上位机协议使用 USART1；VOFA 与原 UART 编码器通道均不再参与当前工程。

## 7. 已验证内容

### 7.1 下位机编译

执行：

```powershell
cmake --build --preset Debug
```

结果：

- 编译成功。
- 链接成功。
- 生成 `FOC_ZT.elf`。

剩余警告：

```text
Lib/Src/encoder.c: unused variable 'turn_count'
```

该警告是原有 UART 编码器解析代码中的未使用变量，和本次协议通信新增逻辑无关。

### 7.2 上位机语法检查

执行：

```powershell
python -m py_compile turntable_control.py
```

结果：

- 语法检查通过。

注意：这一步生成了 `__pycache__/turntable_control.cpython-312.pyc` 缓存文件，不影响运行。

## 8. 建议下一步调试顺序

1. 确认上位机串口连接参数为 `115200, 8N1`。
2. 先测试使能/断能按钮，看电机使能脚是否响应。
3. 再测试主界面双轴目标角度发送，确认上位机能收到 12 字节状态回传。
4. 测试手动左右按钮，观察方位轴目标是否变化。
5. 测试 PID 下发，确认 `position_pid_inst/speed_pid_inst/iq_pid_inst/id_pid_inst` 是否被更新。
6. 最后再做扫频和功能开关，因为这两部分目前只是协议接收骨架，还没有完整业务动作。

## 9. 2026-07-29 电流环 D/Q 轴 PID 显示更新

本次把电流环 PID 从“只显示/下发 q 轴”扩展为“同时显示/下发 q 轴和 d 轴”。

- 下位机 `Lib/Src/turntable_comm.c`：保留 `loop=0x00` 对应 `iq_pid_inst`，新增 `loop=0x03` 对应 `id_pid_inst`。
- 下位机 PID 查询扩展：当上位机发送 `loop=0xFF` 查询全部 PID 时，现在会依次回传 `0x00(Iq)`, `0x03(Id)`, `0x01(speed)`, `0x02(position)`。
- 上位机 `turntable_control.py`：手动 PID 面板打开后，原电流环列改为 `Q轴电流环`，新增 `D轴电流环` 三个输入框。
- 上位机发送 PID：选择“电流环”并点击发送时，会连续发送两帧，分别写入 `loop=0x00` 的 q 轴电流环和 `loop=0x03` 的 d 轴电流环。

协议符合性说明：Word 原协议没有定义“读取当前 PID 参数”和“电流环 D/Q 轴分别查询”的标准字段，因此 `0x05 0x10/0x11` 仍属于当前工程内部扩展；原有 PID 设置帧格式和帧头、帧尾、float 小端、校验规则保持不变。

## 10. 2026-07-29 手动控制面板步进更新

- 上位机 `turntable_control.py`：手动控制面板方向键步进从 `0.1°` 改为 `5°`。
- 左右水平轴调整改为基于面板内部保存的目标角度连续累加，不再每次都拿回传实际角度作为基准，避免实际角度刷新慢时连续点击无变化。
- 水平轴角度按 `0~360°` 循环处理。
- `归位` 按钮现在只发送水平/方位轴单轴角度设置帧，目标角度为 `245°`，不再同时把俯仰轴设置为 `0°`。

## 11. 2026-08-01 ADC双端点采样诊断

- TIM1在中心对齐PWM的两个端点分别触发一组完整ADC注入序列，序列频率为40 kHz。
- ADC层将相邻两组U/V/W/VBUS数据平均后，只执行一次20 kHz FOC控制。
- 电流零点使用4096组样本做浮点平均，避免整数截断造成亚计数偏差。
- 当前Q轴参考电流限制为±2.0 A；该限制同时参与速度PID外层抗积分饱和。

新增只查询不周期发送的ADC诊断帧：

```text
查询：A5 5A 05 19 1E 0D 0A
回报：A5 5A 05 19 flags offset_u offset_v offset_w
      endpoint_delta_u endpoint_delta_v endpoint_delta_w
      adc_sequence_count foc_update_count checksum 0D 0A
```

回报帧共40字节，浮点数与32位计数均为小端格式。`flags`位0表示零点校准有效，位1表示当前已收到第一端点、正在等待第二端点。

## 12. 2026-08-01 机械堵转诊断与限幅跟踪

机械卡滞时，速度PID原始输出可能继续增加，而实验层总Iq已被限制为
±2.0 A。现已将总Iq未能执行的部分折算回速度PID积分状态，避免卡滞
解除后携带隐藏积分突然加速。

堵转判定条件为：存在至少0.2 RPM的运动目标、实际速度不高于0.2 RPM、
总Iq目标达到限幅的95%，并持续500 ms。堵转只作为诊断状态上报，
不会自动断能，也不会突破总Iq限制；实际速度达到0.3 RPM后解除状态。

新增只查询不周期发送的堵转诊断帧：

```text
查询：A5 5A 05 1A 1F 0D 0A
回报：A5 5A 05 1A flags
      speed_target speed_actual iq_pid_raw iq_target iq_measured
      last_stall_angle stall_duration_ms stall_event_count
      checksum 0D 0A
```

回报共40字节。`flags`位0表示当前已确认堵转，位1表示总Iq限幅正在
修正速度PID积分。六个浮点数单位依次为RPM、RPM、A、A、A、度，
最后两个字段为小端32位无符号整数。
