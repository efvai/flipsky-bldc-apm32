#ifndef DATATYPES_H_
#define DATATYPES_H_
#include <stdint.h>
#include <stdbool.h>

typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP,
	CONTROL_MODE_OPENLOOP_PHASE,
	CONTROL_MODE_OPENLOOP_DUTY,
	CONTROL_MODE_OPENLOOP_DUTY_PHASE,
	CONTROL_MODE_NONE
} mc_control_mode;

typedef enum {
	FOC_SENSOR_MODE_SENSORLESS = 0,
	FOC_SENSOR_MODE_ENCODER,
	FOC_SENSOR_MODE_HALL,
	FOC_SENSOR_MODE_HFI,
	FOC_SENSOR_MODE_HFI_START,
	FOC_SENSOR_MODE_HFI_V2,
	FOC_SENSOR_MODE_HFI_V3,
	FOC_SENSOR_MODE_HFI_V4,
	FOC_SENSOR_MODE_HFI_V5
} mc_foc_sensor_mode;

// FOC current controller decoupling mode.
typedef enum {
	FOC_CC_DECOUPLING_DISABLED = 0,
	FOC_CC_DECOUPLING_CROSS,
	FOC_CC_DECOUPLING_BEMF,
	FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef enum {
	FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO = 0,
	FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS,
	FOC_CURRENT_SAMPLE_MODE_HIGH_CURRENT
} mc_foc_current_sample_mode;

typedef enum {
	MTPA_MODE_OFF = 0,
	MTPA_MODE_IQ_TARGET,
	MTPA_MODE_IQ_MEASURED
} MTPA_MODE;

typedef enum {
	FOC_SPEED_SRC_CORRECTED = 0,
	FOC_SPEED_SRC_OBSERVER,
} FOC_SPEED_SRC;

typedef struct {
	// Limits
	float l_max_duty;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;

	// FOC
	float foc_current_kp;
	float foc_current_ki;
	float foc_f_zv;
	float foc_dt_us;
	float foc_motor_r;
	float foc_motor_l;
	float foc_motor_ld_lq_diff;
	float foc_motor_flux_linkage;
	mc_foc_sensor_mode foc_sensor_mode;
	float foc_encoder_offset;
	bool foc_encoder_inverted;
	float foc_encoder_ratio;
	bool foc_temp_comp;
	float foc_current_filter_const;
	mc_foc_cc_decoupling_mode foc_cc_decoupling;
	mc_foc_current_sample_mode foc_current_sample_mode;
	float foc_sl_erpm_start;
	float foc_sl_erpm;
	float foc_offsets_current[3];
	float foc_offsets_voltage[3];
	float foc_offsets_voltage_undriven[3];
	MTPA_MODE foc_mtpa_mode;
	// Field Weakening
	float foc_fw_current_max;
	float foc_fw_duty_start;
	float foc_fw_ramp_time;
	float foc_fw_q_current_factor;
	FOC_SPEED_SRC foc_speed_soure;
	bool foc_short_ls_on_zero_duty;

	// Setup info
	uint8_t si_motor_poles;
} mc_configuration;

#endif