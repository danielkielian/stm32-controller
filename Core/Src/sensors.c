/*
 * sensors.c
 *
 *  Created on: 15 cze 2021
 *      Author: DanielKielian
 */

#include "main.h"
#include "sensors.h"
#include "math.h"

static ThrottleSensor_t throttle_sens;
static BatterySensor_t battery_sens;
static CurrentSensor_t current_sens;
static RpmSensor_t rpm_sens;

//Returns throttle postion 0-1000[-]
float CalculateThrottle(uint16_t* adc_value)
{
	float out = (float)(*adc_value*throttle_sens.gain);
	if(out<0)
	{
		out = 0;
	} else if (out > 1000) {
		out = 1000;
	}
	return out;
}
//Returns battery voltage [V]
float CalculateBatteryVoltage(uint16_t* adc_value)
{
	float voltage_devider = (float)*adc_value*POWER_SUPPLY_VOLTAGE/ADC_RESOLUTION;
	return voltage_devider*battery_sens.gain;
}
//Returns battery level [%]
uint8_t GetBatteryPercentage(float* voltage)
{
	float reltive_voltage;	//[0,1]
	reltive_voltage = (*voltage-battery_sens.min_voltage)/(battery_sens.difference_voltage);
	if(reltive_voltage > 1)
	{
		reltive_voltage = 1;
	} else if (reltive_voltage < 0)
	{
		reltive_voltage = 0;
	}
	return (uint8_t)(reltive_voltage * 100);
}
//Returns current[A]
float CalculateCurrent(uint16_t* adc_value)
{
	float sensor_signal = (float)*adc_value*POWER_SUPPLY_VOLTAGE/ADC_RESOLUTION;
	return (fabsf(sensor_signal - current_sens.offset))/current_sens.gain;
}
// Return RPM and clear counter
uint16_t GetRPM(void)
{
	uint32_t rpm = (*rpm_sens.counter * rpm_sens.gain) / rpm_sens.nbr_of_blades;
	*rpm_sens.counter = 0;
	return rpm;
}

/* Set default sensor's parameters
 * cnt - pointer to timer CNT register
 * timer_period[ms] - time between RPM calculations
 */
void SensorsInit(volatile uint32_t* cnt,uint16_t timer_period)
{
	throttle_sens.gain = 1000.0/ADC_RESOLUTION;		//1000.0 - throttle resolution
	battery_sens.gain = (10.0+47.0)/10.0;		//R1 = 47K, R2=10K
	battery_sens.max_voltage = 12.3;
	battery_sens.min_voltage = 3.6 * 3;
	battery_sens.difference_voltage = battery_sens.max_voltage - battery_sens.min_voltage;
	current_sens.gain = 0.1;					//Vout = Vcc/2[V] +/- 0.1[V/A]*I[A]
	current_sens.offset = 4.66/2.0;				//Vout = Vcc/2[V] +/- 0.1[V/A]*I[A]
	rpm_sens.nbr_of_blades = 2;
	rpm_sens.counter = cnt;
	rpm_sens.gain = 60*1000 / timer_period;
}
