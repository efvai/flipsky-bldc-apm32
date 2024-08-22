#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include "main.h"
#include "datatypes.h"

void mcpwm_foc_init(mc_configuration *conf);
void mcpwm_foc_tim_sample_interrupt_handler();
void mcpwm_foc_adc_interrupt_handler();

float mcpwm_foc_get_phase(void);
void mcpwm_foc_get_current_offsets(volatile float *curr0_offset,
                                   volatile float *curr1_offset,
                                   volatile float *curr2_offset);
void mcpwm_foc_set_current_offsets(volatile float curr0_offset,
                                   volatile float curr1_offset,
                                   volatile float curr2_offset);

// External variables
extern volatile uint16_t ADC_Value[];
extern volatile float ADC_curr_norm_value[];
extern volatile float ADC_curr_raw[];

#endif