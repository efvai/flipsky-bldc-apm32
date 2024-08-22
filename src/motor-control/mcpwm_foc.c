#include "mcpwm_foc.h"
#include "hw.h"
#include "utils_math.h"
#include "foc_math.h"
#include "virtual_motor.h"

/* Macro */
#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
    TIM1->CR1 |= TIM_CR1_UDIS;                    \
    TIM1->CCR1 = duty1;                           \
    TIM1->CCR2 = duty3;                           \
    TIM1->CCR3 = duty2;                           \
    TIM1->CR1 &= ~TIM_CR1_UDIS;

/* Extern Variables */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

/* Global Variables*/

volatile uint16_t ADC_Value[HW_ADC_CHANNELS];
volatile float ADC_curr_norm_value[6];
volatile float ADC_curr_raw[6];

typedef struct
{
    float input_voltage_filtered;

    float motor_current_sum;
    float input_current_sum;
    float motor_current_iterations;
    float input_current_iterations;
    float motor_id_sum;
    float motor_iq_sum;
    float motor_id_iterations;
    float motor_iq_iterations;
    float motor_vd_sum;
    float motor_vq_sum;
    float motor_vd_iterations;
    float motor_vq_iterations;

    uint32_t cycles_running;
} motor_if_state_t;

/* Private Variables */

static volatile motor_if_state_t g_motor;
static volatile motor_all_state_t g_motor_foc;
static volatile bool g_init_done;

/* Private Functions*/

static void timer_reinit(int f_zv);
static void stop_pwm_hw();
static void start_pwm_hw();
static void control_current(volatile motor_all_state_t *motor, float dt);
static void update_valpha_vbeta(volatile motor_all_state_t *motor, float mod_alpha, float mod_beta);

/* Function definitions */

void mcpwm_foc_init(mc_configuration *conf)
{
    g_init_done = false;
    g_motor_foc.m_conf = conf;
    g_motor_foc.m_state = MC_STATE_OFF;
    g_motor_foc.m_control_mode = CONTROL_MODE_NONE;
    g_motor_foc.m_hall_dt_diff_last = 1.0f;
    g_motor_foc.m_hall_dt_diff_now = 1.0f;
    g_motor_foc.m_ang_hall_int_prev = -1;
    // TODO: foc_precalc_values

    virtual_motor_init(conf);

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

    /* ADC1 Init */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = HW_ADC_NBR_CONV;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode */
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC2 Init*/
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = HW_ADC_NBR_CONV;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC3 Init*/
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ENABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = HW_ADC_NBR_CONV;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = 3;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 4;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable ADCs*/
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADC_Start(&hadc3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)ADC_Value, HW_ADC_CHANNELS) != HAL_OK)
    {
        Error_Handler();
    }

    /* Disable Half transfer itr, need to investigate */
    hdma_adc1.Instance->CR &= ~DMA_IT_HT;
    /* Register DMA Callback */

    timer_reinit(15000);

    ENABLE_GATE();
    DCCAL_OFF();
    /* Wait for Input Voltage to settle */

    g_init_done = true;
}

/* Getters */

float mcpwm_foc_get_phase(void) {
    float angle = RAD2DEG_f(g_motor_foc.m_motor_state.phase);
	utils_norm_angle(&angle);
	return angle;
}

void mcpwm_foc_get_current_offsets(volatile float *curr0_offset,
                                   volatile float *curr1_offset,
                                   volatile float *curr2_offset)
{
    *curr0_offset = g_motor_foc.m_conf->foc_offsets_current[0];
    *curr1_offset = g_motor_foc.m_conf->foc_offsets_current[1];
    *curr2_offset = g_motor_foc.m_conf->foc_offsets_current[2];
}

/* Setters */

void mcpwm_foc_set_current_offsets(volatile float curr0_offset,
                                   volatile float curr1_offset,
                                   volatile float curr2_offset)
{
    g_motor_foc.m_conf->foc_offsets_current[0] = curr0_offset;
    g_motor_foc.m_conf->foc_offsets_current[1] = curr1_offset;
    g_motor_foc.m_conf->foc_offsets_current[2] = curr2_offset;
}

/* Handlers */

void mcpwm_foc_tim_sample_interrupt_handler()
{
    if (g_init_done)
    {
        /* Generate COM event for sync*/
        HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);

        virtual_motor_int_handler(g_motor_foc.m_motor_state.v_alpha,
                                  g_motor_foc.m_motor_state.v_beta);
    }
}

void mcpwm_foc_adc_interrupt_handler()
{
    mc_configuration *conf = g_motor_foc.m_conf;
    float dt = 1.0 / (conf->foc_f_zv / 2.0);

    float curr0 = GET_CURRENT1();
    float curr1 = GET_CURRENT2();
    float curr2 = GET_CURRENT3();

    // Store raw ADC readings
    g_motor_foc.m_currents_adc[0] = curr0;
    g_motor_foc.m_currents_adc[1] = curr1;
    g_motor_foc.m_currents_adc[2] = curr2;

    // Shift to midpoint using offset (should be close to 2048)
    curr0 -= conf->foc_offsets_current[0];
    curr1 -= conf->foc_offsets_current[1];
    curr2 -= conf->foc_offsets_current[2];

    // Store midshifted raw ADC readings for raw sampling mode.
    ADC_curr_raw[0] = curr0;
    ADC_curr_raw[1] = curr1;
    ADC_curr_raw[2] = curr2;

    // Scale to AMPs
    curr0 *= FAC_CURRENT;
    curr1 *= FAC_CURRENT;
    curr2 *= FAC_CURRENT;

    // Store the currents for sampling
    ADC_curr_norm_value[0] = curr0;
    ADC_curr_norm_value[1] = curr1;
    ADC_curr_norm_value[2] = curr2;
    float ia = curr0;
    float ib = curr1;
    float ic = curr2;

    UTILS_LP_FAST(g_motor_foc.m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);
    volatile float enc_ang = 0;
    volatile bool encoder_using = false;

    if (virtual_motor_is_connected())
    {
        if (conf->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER)
        {
            enc_ang = virtual_motor_get_angle_deg();
            encoder_using = true;
        }
    }

    if (encoder_using)
    {
        float phase_tmp = enc_ang;
        if (conf->foc_encoder_inverted)
        {
            phase_tmp = 360.0 - phase_tmp;
        }
        phase_tmp *= conf->foc_encoder_ratio;
        phase_tmp -= conf->foc_encoder_offset;
        utils_norm_angle((float *)&phase_tmp);
        g_motor_foc.m_phase_now_encoder = DEG2RAD_f(phase_tmp);
    }

    if (g_motor_foc.m_state != MC_STATE_RUNNING)
    {
        // The current is 0 when the motor is undriven
        g_motor_foc.m_motor_state.i_alpha = 0.0;
        g_motor_foc.m_motor_state.i_beta = 0.0;
        g_motor_foc.m_motor_state.id = 0.0;
        g_motor_foc.m_motor_state.iq = 0.0;
        g_motor_foc.m_motor_state.id_filter = 0.0;
        g_motor_foc.m_motor_state.iq_filter = 0.0;
        g_motor_foc.m_duty_i_term = 0.0;
        g_motor_foc.m_motor_state.i_bus = 0.0;
        g_motor_foc.m_motor_state.i_abs = 0.0;
        g_motor_foc.m_motor_state.i_abs_filter = 0.0;

        /* TODO: Observer and HFI*/

        float s = g_motor_foc.m_motor_state.phase_sin;
        float c = g_motor_foc.m_motor_state.phase_cos;

        // Park transform
        float vd_tmp = c * g_motor_foc.m_motor_state.v_alpha + s * g_motor_foc.m_motor_state.v_beta;
        float vq_tmp = c * g_motor_foc.m_motor_state.v_beta - s * g_motor_foc.m_motor_state.v_alpha;

        UTILS_NAN_ZERO(g_motor_foc.m_motor_state.vd);
        UTILS_NAN_ZERO(g_motor_foc.m_motor_state.vq);

        UTILS_LP_FAST(g_motor_foc.m_motor_state.vd, vd_tmp, 0.2);
        UTILS_LP_FAST(g_motor_foc.m_motor_state.vq, vq_tmp, 0.2);

        // Set the current controller integrator to the BEMF voltage to avoid
        // a current spike when the motor is driven again. Notice that we have
        // to take decoupling into account.
        g_motor_foc.m_motor_state.vd_int = g_motor_foc.m_motor_state.vd;
        g_motor_foc.m_motor_state.vq_int = g_motor_foc.m_motor_state.vq;

        if (conf->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
            conf->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF)
        {
            g_motor_foc.m_motor_state.vq_int -= g_motor_foc.m_pll_speed * conf->foc_motor_flux_linkage;
        }

        // Update corresponding modulation
        /* voltage_normalize = 1/(2/3*V_bus) */
        const float voltage_normalize = 1.5 / g_motor_foc.m_motor_state.v_bus;

        g_motor_foc.m_motor_state.mod_d = g_motor_foc.m_motor_state.vd * voltage_normalize;
        g_motor_foc.m_motor_state.mod_q = g_motor_foc.m_motor_state.vq * voltage_normalize;
        UTILS_NAN_ZERO(g_motor_foc.m_motor_state.mod_q_filter);
        UTILS_LP_FAST(g_motor_foc.m_motor_state.mod_q_filter, g_motor_foc.m_motor_state.mod_q, 0.2);
        utils_truncate_number_abs((float *)&g_motor_foc.m_motor_state.mod_q_filter, 1.0);
    }
    else
    {
        /* Motor Running State */

        if (conf->foc_current_sample_mode == FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS)
        {
            // Full Clarke Transform
            g_motor_foc.m_motor_state.i_alpha = (2.0 / 3.0) * ia - (1.0 / 3.0) * ib - (1.0 / 3.0) * ic;
            g_motor_foc.m_motor_state.i_beta = ONE_BY_SQRT3 * ib - ONE_BY_SQRT3 * ic;
        }
        else
        {
            // Clarke transform assuming balanced currents
            g_motor_foc.m_motor_state.i_alpha = ia;
            g_motor_foc.m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
        }

        g_motor_foc.m_i_alpha_sample_with_offset = g_motor_foc.m_motor_state.i_alpha;
        g_motor_foc.m_i_beta_sample_with_offset = g_motor_foc.m_motor_state.i_beta;

        if (g_motor_foc.m_i_alpha_beta_has_offset)
        {
            g_motor_foc.m_motor_state.i_alpha = 0.5 * (g_motor_foc.m_motor_state.i_alpha + g_motor_foc.m_i_alpha_sample_next);
            g_motor_foc.m_motor_state.i_beta = 0.5 * (g_motor_foc.m_motor_state.i_beta + g_motor_foc.m_i_beta_sample_next);
            g_motor_foc.m_i_alpha_beta_has_offset = false;
        }

        const float duty_now = g_motor_foc.m_motor_state.duty_now;
        const float duty_abs = fabsf(duty_now);
        //const float vq_now = g_motor_foc.m_motor_state.vq;
        //const float speed_fast_now = g_motor_foc.m_pll_speed;

        float id_set_tmp = g_motor_foc.m_id_set;
        float iq_set_tmp = g_motor_foc.m_iq_set;
        g_motor_foc.m_motor_state.max_duty = conf->l_max_duty;

        /* TODO: limit current at CONTROL_MODE_CURRENT_BRAKE */
        UTILS_LP_FAST(g_motor_foc.m_duty_abs_filtered, duty_abs, 0.01);
        utils_truncate_number_abs((float *)&g_motor_foc.m_duty_abs_filtered, 1.0);

        UTILS_LP_FAST(g_motor_foc.m_duty_filtered, duty_now, 0.01);
        utils_truncate_number_abs((float *)&g_motor_foc.m_duty_filtered, 1.0);

        //float duty_set = g_motor_foc.m_duty_cycle_set;

        /* TODO: Something with braking and duty control mode */

        // Set motor phase
        {
            // foc_observer_update(g_motor_foc.m_motor_state.v_alpha, g_motor_foc.m_motor_state.v_beta,
            //                     g_motor_foc.m_motor_state.i_alpha, g_motor_foc.m_motor_state.i_beta,
            //                     dt, &(g_motor_foc.m_observer_state), &g_motor_foc.m_phase_now_observer, &g_motor_foc);

            // Compensate from the phase lag caused by the switching frequency. This is important for motors
            // that run on high ERPM compared to the switching frequency.
            // g_motor_foc.m_phase_now_observer += g_motor_foc.m_pll_speed * dt * (0.5 + conf->foc_observer_offset);
            // utils_norm_angle_rad((float *)&g_motor_foc.m_phase_now_observer);
            switch (conf->foc_sensor_mode)
            {
            case FOC_SENSOR_MODE_ENCODER:
                if (/*encoder_index_found() ||*/ virtual_motor_is_connected())
                {
                    // g_motor_foc.m_motor_state.phase = foc_correct_encoder(
                    //     g_motor_foc.m_phase_now_observer,
                    //     g_motor_foc.m_phase_now_encoder,
                    //     g_motor_foc.m_speed_est_fast,
                    //     conf->foc_sl_erpm,
                    //     &g_motor_foc);
                }
                else
                {
                    // Rotate the motor in open loop if the index isn't found.
                    g_motor_foc.m_motor_state.phase = g_motor_foc.m_phase_now_encoder_no_index;
                }

                if (!g_motor_foc.m_phase_override && g_motor_foc.m_control_mode != CONTROL_MODE_OPENLOOP_PHASE)
                {
                    id_set_tmp = 0.0;
                }
                break;
            default:
                break;
            }
            utils_fast_sincos_better(g_motor_foc.m_motor_state.phase,
                                     (float *)&g_motor_foc.m_motor_state.phase_sin,
                                     (float *)&g_motor_foc.m_motor_state.phase_cos);
        }

        // Apply MTPA. See: https://github.com/vedderb/bldc/pull/179
        const float ld_lq_diff = conf->foc_motor_ld_lq_diff;
        if (conf->foc_mtpa_mode != MTPA_MODE_OFF && ld_lq_diff != 0.0)
        {
            const float lambda = conf->foc_motor_flux_linkage;

            float iq_ref = iq_set_tmp;
            if (conf->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED)
            {
                iq_ref = utils_min_abs(iq_set_tmp, g_motor_foc.m_motor_state.iq_filter);
            }

            id_set_tmp = (lambda - sqrtf(SQ(lambda) + 8.0 * SQ(ld_lq_diff * iq_ref))) / (4.0 * ld_lq_diff);
            iq_set_tmp = SIGN(iq_set_tmp) * sqrtf(SQ(iq_set_tmp) - SQ(id_set_tmp));
        }

        const float mod_q = g_motor_foc.m_motor_state.mod_q_filter;

        // Running FW from the 1 khz timer seems fast enough.
        id_set_tmp -= g_motor_foc.m_i_fw_set;
        iq_set_tmp -= SIGN(mod_q) * g_motor_foc.m_i_fw_set * conf->foc_fw_q_current_factor;

        // Apply current limits
        // TODO: Consider D axis current for the input current as well. Currently this is done using
        // l_in_current_map_start in update_override_limits.
        if (mod_q > 0.001)
        {
            utils_truncate_number(&iq_set_tmp, conf->lo_in_current_min / mod_q, conf->lo_in_current_max / mod_q);
        }
        else if (mod_q < -0.001)
        {
            utils_truncate_number(&iq_set_tmp, conf->lo_in_current_max / mod_q, conf->lo_in_current_min / mod_q);
        }

        if (mod_q > 0.0)
        {
            utils_truncate_number(&iq_set_tmp, conf->lo_current_min, conf->lo_current_max);
        }
        else
        {
            utils_truncate_number(&iq_set_tmp, -conf->lo_current_max, -conf->lo_current_min);
        }

        float current_max_abs = fabsf(utils_max_abs(conf->lo_current_max, conf->lo_current_min));
        utils_truncate_number_abs(&id_set_tmp, current_max_abs);
        utils_truncate_number_abs(&iq_set_tmp, sqrtf(SQ(current_max_abs) - SQ(id_set_tmp)));

        g_motor_foc.m_motor_state.id_target = id_set_tmp;
        g_motor_foc.m_motor_state.iq_target = iq_set_tmp;

        /* Run very important function */
        control_current(&g_motor_foc, dt);
    }

    // Calculate duty cycle
    g_motor_foc.m_motor_state.duty_now = SIGN(g_motor_foc.m_motor_state.vq) *
                                         NORM2_f(g_motor_foc.m_motor_state.mod_d, g_motor_foc.m_motor_state.mod_q) * TWO_BY_SQRT3;

    //float phase_for_speed_est = 0.0;
    // switch (conf->foc_speed_soure)
    // {
    // case FOC_SPEED_SRC_CORRECTED:
    //     phase_for_speed_est = g_motor_foc.m_motor_state.phase;
    //     break;
    // case FOC_SPEED_SRC_OBSERVER:
    //     phase_for_speed_est = g_motor_foc.m_phase_now_observer;
    //     break;
    // };

    // Run PLL for speed estimation
    //foc_pll_run(phase_for_speed_est, dt, &g_motor_foc.m_pll_phase, &g_motor_foc.m_pll_speed, conf);

    // Low latency speed estimation, for e.g. HFI and speed control.
    {
        // float diff = utils_angle_difference_rad(phase_for_speed_est, g_motor_foc.m_phase_before_speed_est);
        // utils_truncate_number(&diff, -M_PI / 3.0, M_PI / 3.0);

        // UTILS_LP_FAST(g_motor_foc.m_speed_est_fast, diff / dt, 0.01);
        // UTILS_NAN_ZERO(g_motor_foc.m_speed_est_fast);

        // UTILS_LP_FAST(g_motor_foc.m_speed_est_faster, diff / dt, 0.2);
        // UTILS_NAN_ZERO(g_motor_foc.m_speed_est_faster);

        // float diff_corr = utils_angle_difference_rad(g_motor_foc.m_motor_state.phase, g_motor_foc.m_phase_before_speed_est_corrected);
        // utils_truncate_number(&diff_corr, -M_PI / 3.0, M_PI / 3.0);

        // UTILS_LP_FAST(g_motor_foc.m_speed_est_fast_corrected, diff_corr / dt, 0.01);
        // UTILS_NAN_ZERO(g_motor_foc.m_speed_est_fast_corrected);

        // // pll wind-up protection
        // utils_truncate_number_abs((float *)&g_motor_foc.m_pll_speed, fabsf(g_motor_foc.m_speed_est_fast) * 3.0);

        // g_motor_foc.m_phase_before_speed_est = phase_for_speed_est;
        // g_motor_foc.m_phase_before_speed_est_corrected = g_motor_foc.m_motor_state.phase;
    }
}

/* HAL Callbacks */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mcpwm_foc_adc_interrupt_handler();

    /* interface timer isr */
    const float input_voltage = GET_INPUT_VOLTAGE();
    UTILS_LP_FAST(g_motor.input_voltage_filtered, input_voltage, 0.02);

    /* TODO: Check voltage faults */

    /* Get some values to prevent overwritting from rtos threads */
    // mc_state state = g_motor_foc.m_state;
    // float current = SIGN(g_motor_foc.m_motor_state.vq *
    //                      g_motor_foc.m_motor_state.iq) *
    //                 g_motor_foc.m_motor_state.i_abs;
    // float current_filtered = SIGN(g_motor_foc.m_motor_state.vq *
    //                               g_motor_foc.m_motor_state.iq_filter) *
    //                          g_motor_foc.m_motor_state.i_abs_filter;
    // float current_in_filtered = g_motor_foc.m_motor_state.i_bus;
    // float abs_current = g_motor_foc.m_motor_state.i_abs;
    // float abs_current_filtered = g_motor_foc.m_motor_state.i_abs_filter;

    // if (state == MC_STATE_RUNNING)
    // {
    //     g_motor.cycles_running++;
    // }
    // else
    // {
    //     g_motor.cycles_running = 0;
    // }

    // g_motor.motor_current_sum += current_filtered;
    // g_motor.input_current_sum += current_in_filtered;
    // g_motor.motor_current_iterations++;
    // g_motor.input_current_iterations++;

    // g_motor.motor_id_sum += g_motor_foc.m_motor_state.id;
    // g_motor.motor_iq_sum += g_motor_foc.m_motor_state.iq;
    // g_motor.motor_id_iterations++;
    // g_motor.motor_iq_iterations++;

    // g_motor.motor_vd_sum += g_motor_foc.m_motor_state.vd;
    // g_motor.motor_vq_sum += g_motor_foc.m_motor_state.vq;
    // g_motor.motor_vd_iterations++;
    // g_motor.motor_vq_iterations++;

    /* TODO: Check Current Fault (ABS) */
    /* TODO: Check DRV Fault code */
    /* TODO: Calculate power */
    /* TODO: Sampling */
}

/* Private Functions */

static void timer_reinit(int f_zv)
{
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM8_CLK_DISABLE();
    __HAL_RCC_TIM2_CLK_DISABLE();

    TIM1->CNT = 0;
    TIM2->CNT = 0;
    TIM8->CNT = 0;

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = (SystemCoreClock / f_zv) - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = TIM1->ARR / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 60;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);

    /* TIM 2 Init*/
    TIM_SlaveConfigTypeDef sSlaveConfigTIM2 = {0};
    TIM_MasterConfigTypeDef sMasterConfigTIM2 = {0};
    TIM_OC_InitTypeDef sConfigOCTIM2 = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xffff;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sSlaveConfigTIM2.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfigTIM2.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfigTIM2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfigTIM2.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfigTIM2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfigTIM2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOCTIM2.OCMode = TIM_OCMODE_PWM1;
    sConfigOCTIM2.Pulse = 250;
    sConfigOCTIM2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOCTIM2.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCTIM2, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCTIM2, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOCTIM2, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    

    stop_pwm_hw();
    // TIMER_UPDATE_SAMP??
}

static void stop_pwm_hw()
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Set Output Compare mode to Forced Inactive for Channel 1
    sConfigOC.OCMode = TIM_OCMODE_FORCED_INACTIVE;
    sConfigOC.Pulse = TIM1->ARR / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);

    // Set Output Compare mode to Forced Inactive for Channel 2
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE);

    // Set Output Compare mode to Forced Inactive for Channel 3
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

    // Generate COM event
    HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_COM);
}

static void start_pwm_hw() {
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = TIM1->ARR / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);

    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);

    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
}

static void control_current(volatile motor_all_state_t *motor, float dt)
{
    volatile motor_state_t *state_m = &motor->m_motor_state;
    volatile mc_configuration *conf_now = motor->m_conf;

    float s = state_m->phase_sin;
    float c = state_m->phase_cos;

    //float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

    // TODO: HFI

    float max_duty = fabsf(state_m->max_duty);
    utils_truncate_number(&max_duty, 0.0, conf_now->l_max_duty);

    // Park transform: transforms the currents from stator to the rotor reference frame
    state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
    state_m->iq = c * state_m->i_beta - s * state_m->i_alpha;

    // Low passed currents are used for less time critical parts, not for the feedback
    UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
    UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

    float d_gain_scale = 1.0;
    // if (conf_now->foc_d_gain_scale_start < 0.99) {
    // 	float max_mod_norm = fabsf(state_m->duty_now / max_duty);
    // 	if (max_duty < 0.01) {
    // 		max_mod_norm = 1.0;
    // 	}
    // 	if (max_mod_norm > conf_now->foc_d_gain_scale_start) {
    // 		d_gain_scale = utils_map(max_mod_norm, conf_now->foc_d_gain_scale_start, 1.0,
    // 				1.0, conf_now->foc_d_gain_scale_max_mod);
    // 		if (d_gain_scale < conf_now->foc_d_gain_scale_max_mod) {
    // 			d_gain_scale = conf_now->foc_d_gain_scale_max_mod;
    // 		}
    // 	}
    // }

    float Ierr_d = state_m->id_target - state_m->id;
    float Ierr_q = state_m->iq_target - state_m->iq;

    float ki = conf_now->foc_current_ki;
    if (conf_now->foc_temp_comp)
    {
        ki = motor->m_current_ki_temp_comp;
    }

    state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
    state_m->vq_int += Ierr_q * (ki * dt);

    // Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
    state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale;
    state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

    // Decoupling. Using feedforward this compensates for the fact that the equations of a PMSM
    // are not really decoupled (the d axis current has impact on q axis voltage and visa-versa):
    //      Resistance  Inductance   Cross terms   Back-EMF   (see www.mathworks.com/help/physmod/sps/ref/pmsm.html)
    // vd = Rs*id   +   Ld*did/dt −  ωe*iq*Lq
    // vq = Rs*iq   +   Lq*diq/dt +  ωe*id*Ld     + ωe*ψm
    float dec_vd = 0.0;
    float dec_vq = 0.0;
    float dec_bemf = 0.0;

    // TODO: CONTROL_MODE_HANDBRAKE decouple

    state_m->vd -= dec_vd; // Negative sign as in the PMSM equations
    state_m->vq += dec_vq + dec_bemf;

    // Calculate the max length of the voltage space vector without overmodulation.
    // Is simply 1/sqrt(3) * v_bus. See https://microchipdeveloper.com/mct5001:start. Adds margin with max_duty.
    float max_v_mag = ONE_BY_SQRT3 * max_duty * state_m->v_bus;

    // Saturation and anti-windup. Notice that the d-axis has priority as it controls field
    // weakening and the efficiency.
    float vd_presat = state_m->vd;
    utils_truncate_number_abs((float *)&state_m->vd, max_v_mag);
    state_m->vd_int += (state_m->vd - vd_presat);

    float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));
    float vq_presat = state_m->vq;
    utils_truncate_number_abs((float *)&state_m->vq, max_vq);
    state_m->vq_int += (state_m->vq - vq_presat);

    utils_saturate_vector_2d((float *)&state_m->vd, (float *)&state_m->vq, max_v_mag);

    // mod_d and mod_q are normalized such that 1 corresponds to the max possible voltage:
    //    voltage_normalize = 1/(2/3*V_bus)
    // This includes overmodulation and therefore cannot be made in any direction.
    // Note that this scaling is different from max_v_mag, which is without over modulation.
    const float voltage_normalize = 1.5 / state_m->v_bus;
    state_m->mod_d = state_m->vd * voltage_normalize;
    state_m->mod_q = state_m->vq * voltage_normalize;
    UTILS_NAN_ZERO(state_m->mod_q_filter);
    UTILS_LP_FAST(state_m->mod_q_filter, state_m->mod_q, 0.2);

    state_m->i_bus = state_m->mod_alpha_measured * state_m->i_alpha + state_m->mod_beta_measured * state_m->i_beta;
    state_m->i_abs = NORM2_f(state_m->id, state_m->iq);
    state_m->i_abs_filter = NORM2_f(state_m->id_filter, state_m->iq_filter);

    update_valpha_vbeta(motor, state_m->mod_alpha_raw, state_m->mod_beta_raw);

    // Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
    state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
    state_m->vq = c * motor->m_motor_state.v_beta - s * motor->m_motor_state.v_alpha;

    bool do_hfi = false;
    if (!do_hfi)
    {
        motor->m_hfi.ind = 0;
        motor->m_hfi.ready = false;
        motor->m_hfi.is_samp_n = false;
        motor->m_hfi.prev_sample = 0.0;
        motor->m_hfi.double_integrator = 0.0;
    }

    // Set output (HW Dependent)
    uint32_t duty1, duty2, duty3, top;
    top = TIM1->ARR;

    // Calculate the duty cycles for all the phases. This also injects a zero modulation signal to
    // be able to fully utilize the bus voltage. See https://microchipdeveloper.com/mct5001:start
    foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, top, &duty1, &duty2, &duty3, (uint32_t *)&state_m->svm_sector);
    TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);

    if (virtual_motor_is_connected() == false) {
		// If all duty cycles are equal the phases should be shorted. Instead of
		// modulating the short we keep all low-side FETs on - that will draw less
		// power and not suffer from dead-time distortion. It also gives more
		// braking torque at low speed.
		if (conf_now->foc_short_ls_on_zero_duty && !do_hfi && duty1 == duty2 && duty2 == duty3) {
			// if (motor->m_pwm_mode != FOC_PWM_FULL_BRAKE) {
			// 	full_brake_hw(motor);
			// }
		} else {
			if (motor->m_pwm_mode != FOC_PWM_ENABLED) {
				start_pwm_hw();
			}
		}
	}
}

static void update_valpha_vbeta(volatile motor_all_state_t *motor, float mod_alpha, float mod_beta)
{
    volatile motor_state_t *state_m = &motor->m_motor_state;
    volatile mc_configuration *conf_now = motor->m_conf;
    float Va, Vb, Vc;

    volatile float *ofs_volt = conf_now->foc_offsets_voltage_undriven;
    if (motor->m_state == MC_STATE_RUNNING)
    {
        ofs_volt = conf_now->foc_offsets_voltage;
    }

    Va = (ADC_V_L1_VOLTS - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vb = (ADC_V_L3_VOLTS - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    Vc = (ADC_V_L2_VOLTS - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;

    // Deadtime compensation
    float s = state_m->phase_sin;
    float c = state_m->phase_cos;
    const float i_alpha_filter = c * state_m->id_filter - s * state_m->iq_filter;
    const float i_beta_filter = c * state_m->iq_filter + s * state_m->id_filter;
    const float ia_filter = i_alpha_filter;
    const float ib_filter = -0.5 * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
    const float ic_filter = -0.5 * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

    // mod_alpha_sign = 2/3*sign(ia) - 1/3*sign(ib) - 1/3*sign(ic)
    // mod_beta_sign  = 1/sqrt(3)*sign(ib) - 1/sqrt(3)*sign(ic)
    const float mod_alpha_filter_sgn = (1.0 / 3.0) * (2.0 * SIGN(ia_filter) - SIGN(ib_filter) - SIGN(ic_filter));
    const float mod_beta_filter_sgn = ONE_BY_SQRT3 * (SIGN(ib_filter) - SIGN(ic_filter));

    const float mod_comp_fact = conf_now->foc_dt_us * 1e-6 * conf_now->foc_f_zv;
    const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
    const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

    mod_alpha -= mod_alpha_comp;
    mod_beta -= mod_beta_comp;

    state_m->va = Va;
    state_m->vb = Vb;
    state_m->vc = Vc;
    state_m->mod_alpha_measured = mod_alpha;
    state_m->mod_beta_measured = mod_beta;

    // v_alpha = 2/3*Va - 1/3*Vb - 1/3*Vc
    // v_beta  = 1/sqrt(3)*Vb - 1/sqrt(3)*Vc
    float v_alpha = (1.0 / 3.0) * (2.0 * Va - Vb - Vc);
    float v_beta = ONE_BY_SQRT3 * (Vb - Vc);

    // Keep the modulation updated so that the filter stays updated
    // even when the motor is undriven.
    if (motor->m_state != MC_STATE_RUNNING)
    {
        /* voltage_normalize = 1/(2/3*V_bus) */
        const float voltage_normalize = 1.5 / state_m->v_bus;

        mod_alpha = v_alpha * voltage_normalize;
        mod_beta = v_beta * voltage_normalize;
    }

    float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

    float filter_const = 1.0;
    if (abs_rpm < 10000.0)
    {
        filter_const = utils_map(abs_rpm, 0.0, 10000.0, 0.01, 1.0);
    }

    float v_mag = NORM2_f(v_alpha, v_beta);
    // The 0.1 * v_mag term below compensates for the filter attenuation as the speed increases.
    // It is chosen by trial and error, so this can be improved.
    UTILS_LP_FAST(state_m->v_mag_filter, v_mag + 0.1 * v_mag * filter_const, filter_const);
    UTILS_LP_FAST(state_m->mod_alpha_filter, mod_alpha, filter_const);
    UTILS_LP_FAST(state_m->mod_beta_filter, mod_beta, filter_const);
    UTILS_NAN_ZERO(state_m->v_mag_filter);
    UTILS_NAN_ZERO(state_m->mod_alpha_filter);
    UTILS_NAN_ZERO(state_m->mod_beta_filter);

    mod_alpha = state_m->mod_alpha_filter;
    mod_beta = state_m->mod_beta_filter;

    if (motor->m_state == MC_STATE_RUNNING)
    {
        state_m->v_alpha = mod_alpha * (2.0 / 3.0) * state_m->v_bus;
        state_m->v_beta = mod_beta * (2.0 / 3.0) * state_m->v_bus;
        state_m->is_using_phase_filters = false;
    } else {
        state_m->v_alpha = v_alpha;
		state_m->v_beta = v_beta;
		state_m->is_using_phase_filters = false;
    }
}