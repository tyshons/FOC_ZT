#ifndef FOC_ZT_EXPERIMENT_FIXED_FEEDFORWARD_H
#define FOC_ZT_EXPERIMENT_FIXED_FEEDFORWARD_H

/* 返回由实机扫频系数生成的位置相关q轴电流前馈，单位为A。 */
float Experiment_FixedFeedforward_CurrentA(float mechanical_angle_rad);

#endif /* FOC_ZT_EXPERIMENT_FIXED_FEEDFORWARD_H */
