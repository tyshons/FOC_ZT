#ifndef FOC_ZT_EXPERIMENT_BASELINE_H
#define FOC_ZT_EXPERIMENT_BASELINE_H

/*
 * 将速度环输出（或叠加实验补偿后的i_q指令）限制在配置的安全范围内。
 * 非有限输入视为故障，返回零电流，避免NaN/Inf继续传入FOC电流环。
 */
float Experiment_Baseline_ClampIq(float iq_command_a);

#endif /* FOC_ZT_EXPERIMENT_BASELINE_H */
