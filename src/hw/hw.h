#ifndef HW_H_
#define HW_H_

#include "main.h"

/*
 * ADC Vector
 *
 * 0:	IN0		SENS3
 * 1:	IN1		SENS2
 * 2:	IN2		SENS1
 * 3:	IN8		CURR2
 * 4:	IN9		CURR1
 * 5:	IN3		TEMP_MOSFET
 * 6:	Vrefint
 * 7:	IN6		ADC_EXT2
 * 8:	IN12	VIN
 * 9:	IN4		TX_SDA_NSS
 * 10:	IN5     ADC_EXT
 * 11:	IN3 	TEMP_MOTOR
 */

#define ENABLE_GATE()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
#define DISABLE_GATE()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
#define DCCAL_ON()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define DCCAL_OFF()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define IS_DRV_FAULT()  (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) != GPIO_PIN_SET)

#define LED_GREEN_ON()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
#define LED_GREEN_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define LED_RED_ON()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)
#define LED_RED_OFF()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)

#define HW_ADC_CHANNELS			12
#define HW_ADC_INJ_CHANNELS		2
#define HW_ADC_NBR_CONV			4

// ADC Indexes
#define ADC_IND_SENS1			2
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			0
#define ADC_IND_CURR1			4
#define ADC_IND_CURR2			3
#define ADC_IND_VIN_SENS		8
#define ADC_IND_EXT		        10
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		5
#define ADC_IND_TEMP_MOTOR		11
#define ADC_IND_VREFINT			6

#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		10.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.001
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4095.0 * V_REG)

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Current
#define GET_CURRENT1()		((float)ADC_Value[ADC_IND_CURR1])
#define GET_CURRENT2()		((float)ADC_Value[ADC_IND_CURR2])
#define GET_CURRENT3()		0

// Current ADC to amperes factor
#define FAC_CURRENT			((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

#define VOLTAGE_TO_ADC_FACTOR	( VIN_R2 / (VIN_R2 + VIN_R1) ) * ( 4096.0 / V_REG )

// Voltage on phase input used for FOC 
#ifndef ADC_V_L1_VOLTS
#define ADC_V_L1_VOLTS				((float)ADC_V_L1 / 4096.0 * V_REG)
#endif
#ifndef ADC_V_L2_VOLTS
#define ADC_V_L2_VOLTS				((float)ADC_V_L2 / 4096.0 * V_REG)
#endif
#ifndef ADC_V_L3_VOLTS
#define ADC_V_L3_VOLTS				((float)ADC_V_L3 / 4096.0 * V_REG)
#endif
#ifndef ADC_V_L4_VOLTS
#define ADC_V_L4_VOLTS				((float)ADC_V_L4 / 4096.0 * V_REG)
#endif
#ifndef ADC_V_L5_VOLTS
#define ADC_V_L5_VOLTS				((float)ADC_V_L5 / 4096.0 * V_REG)
#endif
#ifndef ADC_V_L6_VOLTS
#define ADC_V_L6_VOLTS				((float)ADC_V_L6 / 4096.0 * V_REG)
#endif

// Adc voltage scaling on phases and input
#ifndef ADC_VOLTS_PH_FACTOR
#define ADC_VOLTS_PH_FACTOR		1.0
#endif
#ifndef ADC_VOLTS_INPUT_FACTOR
#define ADC_VOLTS_INPUT_FACTOR	1.0
#endif

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOB
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOB
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		11
#define HW_ENC_TIM				TIM4
#define HW_ENC_TIM_AF			GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource11
#define HW_ENC_EXTI_CH			EXTI15_10_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line11
#define HW_ENC_EXTI_ISR_VEC		EXTI15_10_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM4_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM4_IRQHandler

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			HAL_GPIO_ReadPin(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Setting limits
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 150.0
#define HW_LIM_VIN				6.0, 57.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 110.0

void hw_init_gpio(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);

#endif