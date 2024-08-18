#ifndef HW_H_
#define HW_H_

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
 * 8:	IN12	AN_IN
 * 9:	IN4		TX_SDA_NSS
 * 10:	IN5     ADC_EXT
 * 11:	IN3 	TEMP_MOTOR
 */

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

void hw_init_gpio(void);

#endif