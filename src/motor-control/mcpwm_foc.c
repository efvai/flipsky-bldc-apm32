#include "main.h"
#include "hw.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

uint16_t ADC_value[12];

/* Private */
static void timer_reinit(int f_zv);
static void stop_pwm_hw();
static void start_pwm_hw();


void mcpwm_foc_init()
{
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
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

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
    hadc2.Init.NbrOfConversion = 4;
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
    hadc3.Init.NbrOfConversion = 4;
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
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)ADC_value, 12) != HAL_OK)
    {
        Error_Handler();
    }

    timer_reinit(15000);
}

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
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
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
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    // enable n channels

    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);

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
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);

    // Set Output Compare mode to Forced Inactive for Channel 2
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE);

    // Set Output Compare mode to Forced Inactive for Channel 3
    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

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
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);

    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);

    HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    TIM_CCxChannelCmd(&htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
    TIM_CCxNChannelCmd(&htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
}