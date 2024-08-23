#include "main.h"
#include "hw.h"
#include "mcpwm_foc.h"
#include "virtual_motor.h"
#include "mcconf_default.h"

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();
  
  mc_configuration conf;
  conf.l_max_duty = 0.95;
  conf.lo_current_max = 7;
  conf.lo_current_min = -7;
  conf.lo_in_current_max = 7;
  conf.lo_in_current_min = -7;
  conf.foc_current_kp = 0.01;
  conf.foc_current_ki = 0.0001;
  conf.foc_f_zv = 15000;
  conf.foc_dt_us = MCCONF_FOC_DT_US;
  conf.foc_current_kp = MCCONF_FOC_CURRENT_KP;
  conf.foc_current_ki = MCCONF_FOC_CURRENT_KI;
  conf.foc_motor_r = MCCONF_FOC_MOTOR_R;
  conf.foc_motor_l = MCCONF_FOC_MOTOR_L;
  conf.foc_motor_ld_lq_diff = MCCONF_FOC_MOTOR_LD_LQ_DIFF;
  conf.foc_motor_flux_linkage = MCCONF_FOC_MOTOR_FLUX_LINKAGE;
  conf.foc_sensor_mode = FOC_SENSOR_MODE_ENCODER;
  conf.foc_encoder_ratio =  1;
  conf.foc_encoder_inverted = false;
  conf.foc_encoder_offset = 0;
  conf.foc_current_filter_const = MCCONF_FOC_CURRENT_FILTER_CONST;
  conf.foc_temp_comp = false;
	conf.foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
	conf.foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
	conf.foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;
	conf.foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
	conf.foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
	conf.foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;
	conf.foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
	conf.foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
	conf.foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;
  conf.si_motor_poles = MCCONF_SI_MOTOR_POLES;

  
  
  
  hw_init_gpio();
  mcpwm_foc_init(&conf);
  connect_virtual_motor(0, 1e-6, 24);
  mcpwm_foc_set_current(0.5);

	while (1) {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void) {
  __disable_irq();
  while(1) {

  }
}