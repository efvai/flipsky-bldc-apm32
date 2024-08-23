#include "virtual_motor.h"
#include "main.h"
#include "hw.h"
#include "mcpwm_foc.h"
#include "utils_math.h"

#include "math.h"

extern ADC_HandleTypeDef hadc1;

typedef struct
{
    // constant variables
    float Ts;       // Sample Time in s
    float J;        // Rotor/Load Inertia in Nm*s^2
    int v_max_adc;  // max voltage that ADC can measure
    int pole_pairs; // number of pole pairs ( pole numbers / 2)
    float km;       // constant = 1.5 * pole pairs
    float ld;       // motor inductance in D axis in uHy
    float lq;       // motor inductance in Q axis in uHy

    // non constant variables
    float id;     // Current in d-Direction in Amps
    float id_int; // Integral part of id in Amps
    float iq;     // Current in q-Direction in A
    float me;     // Electrical Torque in Nm
    float we;     // Electrical Angular Velocity in rad/s
    float phi;    // Electrical Rotor Angle in rad
    float sin_phi;
    float cos_phi;
    bool connected; // true => connected; false => disconnected;
    float tsj;      // Ts / J;
    float ml;       // load torque
    float v_alpha;  // alpha axis voltage in Volts
    float v_beta;   // beta axis voltage in Volts
    float va;       // phase a voltage in Volts
    float vb;       // phase b voltage in Volts
    float vc;       // phase c voltage in Volts
    float vd;       // d axis voltage in Volts
    float vq;       // q axis voltage in Volts
    float i_alpha;  // alpha axis current in Amps
    float i_beta;   // beta axis current in Amps
    float ia;       // phase a current in Amps
    float ib;       // phase b current in Amps
    float ic;       // phase c current in Amps
} virtual_motor_t;

static volatile virtual_motor_t virtual_motor;
static volatile float m_curr0_offset_backup;
static volatile float m_curr1_offset_backup;
static volatile float m_curr2_offset_backup;
static volatile mc_configuration *m_conf;

// private functions
static inline void run_virtual_motor_electrical(float v_alpha, float v_beta);
static inline void run_virtual_motor_mechanics(float ml);
static inline void run_virtual_motor(float v_alpha, float v_beta, float ml);
static inline void run_virtual_motor_park_clark_inverse(void);

// Public Functions

/**
 * Virtual motor initialization
 */
void virtual_motor_init(volatile mc_configuration *conf)
{

    virtual_motor_set_configuration(conf);

    // virtual motor variables init
    virtual_motor.connected = false; // disconnected

    virtual_motor.me = 0.0;
    virtual_motor.va = 0.0;
    virtual_motor.vb = 0.0;
    virtual_motor.vc = 0.0;
    virtual_motor.ia = 0.0;
    virtual_motor.ib = 0.0;
    virtual_motor.ic = 0.0;
    virtual_motor.we = 0.0;
    virtual_motor.v_alpha = 0.0;
    virtual_motor.v_beta = 0.0;
    virtual_motor.i_alpha = 0.0;
    virtual_motor.i_beta = 0.0;
    virtual_motor.id_int = 0.0;
    virtual_motor.iq = 0.0;
}

void virtual_motor_set_configuration(volatile mc_configuration *conf)
{
    m_conf = conf;

    // recalculate constants that depend on m_conf
    virtual_motor.pole_pairs = m_conf->si_motor_poles / 2;
    virtual_motor.km = 1.5 * virtual_motor.pole_pairs;

    virtual_motor.Ts = (1.0 / m_conf->foc_f_zv);

    if (m_conf->foc_motor_ld_lq_diff > 0.0)
    {
        virtual_motor.lq = m_conf->foc_motor_l + m_conf->foc_motor_ld_lq_diff / 2;
        virtual_motor.ld = m_conf->foc_motor_l - m_conf->foc_motor_ld_lq_diff / 2;
    }
    else
    {
        virtual_motor.lq = m_conf->foc_motor_l;
        virtual_motor.ld = m_conf->foc_motor_l;
    }
}

/**
 * Virtual motor interrupt handler
 */
void virtual_motor_int_handler(float v_alpha, float v_beta)
{
    if (virtual_motor.connected)
    {
        run_virtual_motor(v_alpha, v_beta, virtual_motor.ml);
        mcpwm_foc_adc_interrupt_handler(NULL, 0);
    }
}

bool virtual_motor_is_connected(void)
{
    return virtual_motor.connected;
}

float virtual_motor_get_angle_deg(void)
{
    return RAD2DEG_f(virtual_motor.phi);
}

// Private Functions

/**
 * void connect_virtual_motor( )
 *
 * -disconnects TIM8 trigger to the ADC:
 * 		mcpwm_foc_adc_int_handler() will be called from TIM8 interrupt
 * 		while virtual motor is connected
 * -sets virtual motor parameters
 *
 * @param ml : torque present at motor axis in Nm
 * @param J: rotor inertia Nm*s^2
 * @param Vbus: Bus voltage in Volts
 */
void connect_virtual_motor(float ml, float J, float Vbus)
{
    if (virtual_motor.connected == false)
    {
        // first send 0.0 current command to make system stop PWM outputs
        // mcpwm_foc_set_current(0.0);
        //  disconnect the ADC triggering from TIM8_CC1
        /* ADC1 Init */
        hadc1.Instance = ADC1;
        hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.ScanConvMode = ENABLE;
        hadc1.Init.ContinuousConvMode = DISABLE;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.NbrOfConversion = HW_ADC_NBR_CONV;
        hadc1.Init.DMAContinuousRequests = ENABLE;
        hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

        if (HAL_ADC_Init(&hadc1) != HAL_OK)
        {
            Error_Handler();
        }

        //save current offsets
		mcpwm_foc_get_current_offsets(&m_curr0_offset_backup,
										&m_curr1_offset_backup,
										&m_curr2_offset_backup);
		//set current offsets to 2048
		mcpwm_foc_set_current_offsets(2048, 2048, 2048);

        //sets virtual motor variables
		ADC_Value[ADC_IND_TEMP_MOS] = 2048;
		ADC_Value[ADC_IND_TEMP_MOTOR] = 2048;

        virtual_motor.phi = DEG2RAD_f(mcpwm_foc_get_phase());
		utils_fast_sincos_better(virtual_motor.phi, (float*)&virtual_motor.sin_phi,
														(float*)&virtual_motor.cos_phi);
        // TODO: disable encoder
    }

    //we load 1 to get the transfer function indirectly
	ADC_Value[ADC_IND_VIN_SENS] = 1;

	float tempVoltage = GET_INPUT_VOLTAGE();
	if(tempVoltage != 0.0){
		ADC_Value[ADC_IND_VIN_SENS] = Vbus / GET_INPUT_VOLTAGE();
	}

	//initialize constants
	virtual_motor.v_max_adc = Vbus;
	virtual_motor.J = J;
	virtual_motor.tsj = virtual_motor.Ts / virtual_motor.J;
	virtual_motor.ml = ml;

	virtual_motor.connected = true;
}

/**
 *  void disconnect_virtual_motor( )
 *
 *	if motor is connected:
 *		-stop motor
 *		-disconnect virtual motor
 *		-connects TIM8 back to the trigger of the ADC peripheral
 */
void disconnect_virtual_motor(void) {

	if(virtual_motor.connected){
		// TODO: mcpwm_foc_set_current(0.0);

		//disconnect virtual motor
		virtual_motor.connected = false;

		//set current offsets back
		mcpwm_foc_set_current_offsets(m_curr0_offset_backup, m_curr1_offset_backup,
															m_curr2_offset_backup);

		//then we reconnect the ADC triggering to TIM8_CC1

        // TODO: init encoder
	}
}

/*
 * Run complete Motor Model
 * @param ml	externally applied load torque in Nm (adidionally to the Inertia)
 */
static inline void run_virtual_motor(float v_alpha, float v_beta, float ml){
	run_virtual_motor_electrical(v_alpha, v_beta);
	run_virtual_motor_mechanics(ml);
	run_virtual_motor_park_clark_inverse();
}

/**
 * Run electrical model of the machine
 *
 * Takes as parameters v_alpha and v_beta,
 * which are outputs from the mcpwm_foc system,
 * representing which voltages the controller tried to set at last step
 *
 * @param v_alpha	alpha axis Voltage in V
 * @param v_beta	beta axis Voltage in V
 */
static inline void run_virtual_motor_electrical(float v_alpha, float v_beta){

	virtual_motor.vd =  virtual_motor.cos_phi * v_alpha + virtual_motor.sin_phi * v_beta;
	virtual_motor.vq =  virtual_motor.cos_phi * v_beta - virtual_motor.sin_phi * v_alpha;

	// d axis current
	virtual_motor.id_int += ((virtual_motor.vd +
								virtual_motor.we *
								virtual_motor.pole_pairs *
								virtual_motor.lq * virtual_motor.iq -
								m_conf->foc_motor_r * virtual_motor.id )
								* virtual_motor.Ts ) / virtual_motor.ld;
    /* Why substracting flux linkage ? */
	virtual_motor.id = virtual_motor.id_int;// - m_conf->foc_motor_flux_linkage / virtual_motor.ld;

	// q axis current
	virtual_motor.iq += (virtual_motor.vq -
						virtual_motor.we *
						virtual_motor.pole_pairs *
						(virtual_motor.ld * virtual_motor.id + m_conf->foc_motor_flux_linkage) -
						m_conf->foc_motor_r * virtual_motor.iq )
						* virtual_motor.Ts / virtual_motor.lq;

	// limit current maximum values
	utils_truncate_number_abs((float *) &(virtual_motor.iq) , (2048 * FAC_CURRENT) );
	utils_truncate_number_abs((float *) &(virtual_motor.id) , (2048 * FAC_CURRENT) );
}

/**
 * Run mechanical side of the machine
 * @param ml	externally applied load torque in Nm
 */
static inline void run_virtual_motor_mechanics(float ml){
	virtual_motor.me =  virtual_motor.km * (m_conf->foc_motor_flux_linkage +
											(virtual_motor.ld - virtual_motor.lq) *
											virtual_motor.id ) * virtual_motor.iq;
	// omega
	virtual_motor.we += virtual_motor.tsj * (virtual_motor.me - ml);

	// phi
	virtual_motor.phi += virtual_motor.we * virtual_motor.Ts;

	// phi limits
	while( virtual_motor.phi > M_PI ){
		virtual_motor.phi -= ( 2 * M_PI);
	}

	while( virtual_motor.phi < -1.0 * M_PI ){
		virtual_motor.phi += ( 2 * M_PI);
	}
}

/**
 * Take the id and iq calculated values and translate them into ADC_Values
 */
static inline void run_virtual_motor_park_clark_inverse( void ) {
	utils_fast_sincos_better( virtual_motor.phi , (float*)&virtual_motor.sin_phi,
													(float*)&virtual_motor.cos_phi );

	//	Park Inverse
	virtual_motor.i_alpha = virtual_motor.cos_phi * virtual_motor.id -
							virtual_motor.sin_phi * virtual_motor.iq;
	virtual_motor.i_beta  = virtual_motor.cos_phi * virtual_motor.iq +
							virtual_motor.sin_phi * virtual_motor.id;

	virtual_motor.v_alpha = virtual_motor.cos_phi * virtual_motor.vd -
							virtual_motor.sin_phi * virtual_motor.vq;
	virtual_motor.v_beta  = virtual_motor.cos_phi * virtual_motor.vq +
							virtual_motor.sin_phi * virtual_motor.vd;

	//	Clark Inverse
	virtual_motor.ia = virtual_motor.i_alpha;
	virtual_motor.ib = -0.5 * virtual_motor.i_alpha + SQRT3_BY_2 * virtual_motor.i_beta;
	virtual_motor.ic = -0.5 * virtual_motor.i_alpha - SQRT3_BY_2 * virtual_motor.i_beta;

	virtual_motor.va = virtual_motor.v_alpha;
	virtual_motor.vb = -0.5 * virtual_motor.v_alpha + SQRT3_BY_2 * virtual_motor.v_beta;
	virtual_motor.vc = -0.5 * virtual_motor.v_alpha - SQRT3_BY_2 * virtual_motor.v_beta;

	//	simulate current samples
	ADC_Value[ ADC_IND_CURR1 ] =  virtual_motor.ia / FAC_CURRENT + 2048;
	ADC_Value[ ADC_IND_CURR2 ] =  virtual_motor.ib / FAC_CURRENT + 2048;

	//	simulate voltage samples
	ADC_Value[ ADC_IND_SENS1 ] = virtual_motor.va * VOLTAGE_TO_ADC_FACTOR + 2048;
	ADC_Value[ ADC_IND_SENS2 ] = virtual_motor.vb * VOLTAGE_TO_ADC_FACTOR + 2048;
	ADC_Value[ ADC_IND_SENS3 ] = virtual_motor.vc * VOLTAGE_TO_ADC_FACTOR + 2048;
}