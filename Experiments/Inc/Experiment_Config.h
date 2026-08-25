#ifndef FOC_ZT_EXPERIMENT_CONFIG_H
#define FOC_ZT_EXPERIMENT_CONFIG_H

/*
 * 物理实验配置。
 *
 * 本文件中的所有参数经电机、逆变器和传感器实机确认前，必须保持功率级关闭。
 */

/* TIM1：170 MHz、中心对齐、4250计数，对应20 kHz PWM周期。 */
#define FOC_CURRENT_LOOP_HZ             20000U
#define FOC_CURRENT_LOOP_PERIOD_S       (1.0f / (float)FOC_CURRENT_LOOP_HZ)
#define FOC_CURRENT_LOOP_PERIOD_US      (1000000U / FOC_CURRENT_LOOP_HZ)

/*
 * ADC在中心对齐PWM的两个端点各完成一次完整注入序列。
 * 相邻两组U/V/W/VBUS采样先求平均，再执行一次20 kHz电流环。
 */
#define FOC_ADC_SEQUENCE_HZ             (FOC_CURRENT_LOOP_HZ * 2U)
#define FOC_ADC_SEQUENCES_PER_CONTROL   2U
#define FOC_ADC_OFFSET_SAMPLE_COUNT     4096U

/* 20 kHz控制数据上的一阶低通，截止频率约1.5 kHz。 */
#define FOC_CURRENT_FILTER_ALPHA        0.375827f

#define FOC_SPEED_LOOP_HZ               4000U
#define FOC_SPEED_LOOP_DIVIDER          (FOC_CURRENT_LOOP_HZ / FOC_SPEED_LOOP_HZ)
#define FOC_SPEED_LOOP_PERIOD_S         (1.0f / (float)FOC_SPEED_LOOP_HZ)
/* 速度环仍按4 kHz运行，但使用2 ms角度窗口抑制静止时的编码器差分噪声。 */
#define FOC_SPEED_ESTIMATOR_WINDOW_UPDATES  8U
/* 4 kHz采样下对应约15 Hz的一阶低通系数。 */
#define FOC_SPEED_FILTER_CUTOFF_HZ      15.0f
/* 保留标称4 kHz下的系数，供配置审查和旧数据对照。 */
#define FOC_SPEED_FILTER_ALPHA          0.023286f

#define FOC_PWM_PERIOD_COUNTS           4250U
#define FOC_PWM_COMPARE_MAX             (FOC_PWM_PERIOD_COUNTS - 1U)
#define FOC_PWM_NEUTRAL_COMPARE         (FOC_PWM_PERIOD_COUNTS / 2U)

/* U13025转台电机参数，以及下位机当前使用的电角度标定值。 */
#define MOTOR_POLE_PAIRS                20.0f
#define MOTOR_NAMEPLATE_POLE_PAIRS      20.0f
#define MOTOR_ELECTRICAL_OFFSET_DEG     96.0f
#define SSI_ENCODER_COUNTS_PER_REV      524288.0f
#define SSI_MAX_STEP_DEG                15.0f
#define SSI_MAX_FRAME_AGE_MS            5U
#define SSI_ENABLE_ACQUIRE_TIMEOUT_MS   100U

/*
 * U13025铭牌参数。
 * 铭牌转矩常数0.28 Nm/A使用线电流有效值，而FOC中的i_q使用相电流峰值，
 * 因此控制器模型采用0.28/sqrt(2) Nm/A_peak。
 *
 * 2.5 kg.cm^2 = 2.5e-4 kg.m^2。
 *
 * 铭牌未提供黏性阻尼；下方初值与仿真一致，必须在实机上重新辨识。
 */
#define MOTOR_NAMEPLATE_TORQUE_CONSTANT_NM_PER_A_RMS  0.28f
#define MOTOR_TORQUE_CONSTANT_NM_PER_A  0.1979899f
#define MOTOR_ROTOR_INERTIA_KGM2        2.5e-4f
#define MOTOR_VISCOUS_DAMPING_NMS       1.0e-3f
#define TURNTABLE_EFFECTIVE_INERTIA_KGM2  0.607284354f

/*
 * 5 RPM正向运行时，两次8圈实机扫频得到的一阶固定前馈电流系数。
 * 系数约定：Iq_ff = gain * (Ic*cos(theta) + Is*sin(theta))。
 * 首次闭环验证只使用25%增益，确认一阶速度纹波下降后再逐步提高。
 */
#define EXPERIMENT_FIXED_FF_ORDER1_COS_CURRENT_A  (-0.0035599f)
#define EXPERIMENT_FIXED_FF_ORDER1_SIN_CURRENT_A  ( 0.0600726f)
#define EXPERIMENT_FIXED_FF_ORDER2_COS_CURRENT_A  (-0.055310f)
#define EXPERIMENT_FIXED_FF_ORDER2_SIN_CURRENT_A  (-0.113731f)
#define EXPERIMENT_FIXED_FF_ORDER4_COS_CURRENT_A  (-0.1546590f)
#define EXPERIMENT_FIXED_FF_ORDER4_SIN_CURRENT_A  (-0.0006816f)
#define EXPERIMENT_FIXED_FF_GAIN                  ( 0.75f)

/*
 * 线性ESO：beta1 = 2*w0，beta2 = w0^2。
 * 首轮实机实验使用较小的补偿增益和独立电流限幅；即使ESO模型参数不准，
 * 也不能让观测器单独占满速度环的总q轴电流额度。
 */
#define PAPER_ESO_BANDWIDTH_RAD_S                30.0f
#define EXPERIMENT_ESO_BANDWIDTH_MIN_RAD_S        5.0f
#define EXPERIMENT_ESO_BANDWIDTH_MAX_RAD_S       60.0f
#define EXPERIMENT_ESO_COMPENSATION_GAIN_DEFAULT  0.20f
#define EXPERIMENT_ESO_COMPENSATION_GAIN_MAX      1.00f
#define EXPERIMENT_ESO_CURRENT_LIMIT_DEFAULT_A     0.30f
#define EXPERIMENT_ESO_CURRENT_LIMIT_MAX_A         2.00f
/* 观测状态允许覆盖完整总电流范围，输出限幅不反向篡改扰动估计。 */
#define EXPERIMENT_ESO_OBSERVER_CURRENT_LIMIT_A    5.00f

/* 与Simulink实验一致的位置域学习前馈参数。 */
#define PAPER_LFF_POSITION_BINS         720U
#define PAPER_LFF_MIN_ORDER             1U
#define PAPER_LFF_MAX_ORDER             40U
#define PAPER_LFF_GAMMA                 0.25f
#define PAPER_LFF_LEAKAGE               1.0f
/*
 * 与论文一致，学习前馈只保留1~40阶位置同步分量。
 * 0阶直流负载不进入学习表，由速度PI积分和ESO承担。
 */
#define PAPER_LFF_TORQUE_LIMIT_NM       0.08f
#define PAPER_LFF_RHO_FORGETTING        0.8f
#define PAPER_LFF_RHO_MIN_PAIRS         4U
#define PAPER_LFF_RHO_THRESHOLD         0.8f
#define PAPER_LFF_AMPLITUDE_MIN_NM      2.0e-4f
#define PAPER_LFF_MAGNITUDE_CV_LIMIT    0.8f
#define PAPER_LFF_MIN_COVERAGE          0.95f

/*
 * 选择性学习对照实验使用的可复现时域随机扰动。
 * 扰动只在模式4/5且学习正在进行时注入；冻结学习后自动撤销。
 */
#define PAPER_LFF_NOISE_DEFAULT_SEED            0x13579BDFUL
#define PAPER_LFF_NOISE_MAX_CURRENT_A           0.20f
#define PAPER_LFF_NOISE_HOLD_UPDATES            80U
#define PAPER_LFF_NOISE_FILTER_ALPHA            0.02f

/*
 * 标称48 V系统的保守调试限值。
 * 只有确认电流采样比例后才可提高电流限制。
 */
#define EXPERIMENT_VBUS_MIN_V           8.0f
#define EXPERIMENT_VBUS_MAX_V           60.0f
#define EXPERIMENT_PHASE_CURRENT_MAX_A  12.0f
/* 首轮采样与电角度复核期间启用保守限流；确认无尖叫后再显式放宽。 */
#define FOC_COMMISSIONING_LIMIT_ENABLED 1U
#define FOC_COMMISSIONING_IQ_LIMIT_A    10.0f
#if FOC_COMMISSIONING_LIMIT_ENABLED
#define EXPERIMENT_IQ_REFERENCE_MAX_A   FOC_COMMISSIONING_IQ_LIMIT_A
#else
#define EXPERIMENT_IQ_REFERENCE_MAX_A   10.0f
#endif
#define EXPERIMENT_VOLTAGE_UTILIZATION  0.55f

/*
 * 非基线模式要求每个ADC注入序列只回调一次，并且每个完整PWM周期只执行一次控制回调。
 * 与之匹配的CubeMX ADC/TIM配置通过审查前，应锁定这些模式。
 *
 * 主机测试通过-DEXPERIMENT_RUNTIME_MODES_ENABLED=1覆盖本宏。
 */
#ifndef EXPERIMENT_RUNTIME_MODES_ENABLED
#define EXPERIMENT_RUNTIME_MODES_ENABLED 1U
#endif

/*
 * 主动固定前馈阶次扫频。
 *
 * 扫频采用保守设置：在实测机械角度的整数倍频率下注入较小的q轴电流。
 * 实验管理器只记录速度响应，不会自动应用辨识出的前馈系数。
 */
#define EXPERIMENT_SWEEP_TARGET_SPEED_RPM                 5.0f
#define EXPERIMENT_SWEEP_SPEED_TOLERANCE_RPM              0.5f
/* 完整一圈的平均转速判据；瞬时速度允许包含位置周期波动。 */
#define EXPERIMENT_SWEEP_REV_MEAN_TOLERANCE_RPM           0.25f
/* 超出硬边界的样本不参与当前圈测量，并暂时撤销扫频注入。 */
#define EXPERIMENT_SWEEP_HARD_SPEED_MIN_RPM               3.5f
#define EXPERIMENT_SWEEP_HARD_SPEED_MAX_RPM               6.5f
#define EXPERIMENT_SWEEP_MIN_POSITION_COVERAGE            PAPER_LFF_MIN_COVERAGE
#define EXPERIMENT_SWEEP_DEFAULT_INJECTION_A              0.03f
#define EXPERIMENT_SWEEP_MAX_INJECTION_A                  0.10f
#define EXPERIMENT_SWEEP_CURRENT_HEADROOM_A               0.05f
#define EXPERIMENT_SWEEP_START_ORDER                      1U
#define EXPERIMENT_SWEEP_END_ORDER                        PAPER_LFF_MAX_ORDER
#define EXPERIMENT_SWEEP_SETTLE_REVOLUTIONS               1U
#define EXPERIMENT_SWEEP_MEASURE_REVOLUTIONS              12U
#define EXPERIMENT_SWEEP_MIN_RESPONSE_RPM_PER_A           1.0e-3f
#define EXPERIMENT_SWEEP_MAX_RECOMMENDED_ORDER_CURRENT_A  0.20f
/* 响应幅值至少达到其标准误差的3倍，才认为注入响应可重复。 */
#define EXPERIMENT_SWEEP_MIN_RESPONSE_SNR                 3.0f
/* 把基线和响应的不确定度折算到前馈电流后，标准误差不得超过30 mA。 */
#define EXPERIMENT_SWEEP_MAX_RECOMMENDED_STANDARD_ERROR_A 0.03f

/*
 * 0：安全默认值，需要从调试器显式使能。
 * 1：不建议在首次调试阶段使用。
 */
#define EXPERIMENT_AUTO_ENABLE          0U

#if (FOC_CURRENT_LOOP_HZ % FOC_SPEED_LOOP_HZ) != 0
#error "FOC速度环频率必须能够整除电流环频率"
#endif

#if FOC_SPEED_ESTIMATOR_WINDOW_UPDATES < 2U
#error "测速差分窗口至少需要两个速度环更新周期"
#endif

#if FOC_ADC_SEQUENCES_PER_CONTROL != 2U
#error "当前双端点采样实现要求每个电流环周期包含两组ADC序列"
#endif

#if EXPERIMENT_SWEEP_MEASURE_REVOLUTIONS < 2U
#error "固定前馈扫频至少需要两圈测量数据才能计算样本方差"
#endif

#if (PAPER_LFF_MIN_ORDER < 1U) || \
    (PAPER_LFF_MIN_ORDER > PAPER_LFF_MAX_ORDER)
#error "论文学习前馈只能学习1~M阶非零位置谐波"
#endif

#endif /* FOC_ZT_EXPERIMENT_CONFIG_H */
