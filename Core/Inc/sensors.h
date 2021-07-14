/*
 * sensors.h
 *
 *  Created on: 15 cze 2021
 *      Author: DanielKielian
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#define ADC_RESOLUTION	4095.0f
#define POWER_SUPPLY_VOLTAGE	2.957f	//[V]

typedef struct throttle_sensor_struct {
	float gain;
} ThrottleSensor_t;

typedef struct battery_sensor_struct {
	//Associated with voltage devider
	float gain;

	//Associated with % battery
	float max_voltage;
	float min_voltage;
	float difference_voltage;
} BatterySensor_t;

typedef struct current_sensor_struct {
	float gain;
	float offset;
} CurrentSensor_t;

typedef struct rpm_sensor_struct {
	uint8_t nbr_of_blades;
	volatile uint32_t* counter;
	uint16_t gain;
} RpmSensor_t;

float CalculateThrottle(uint16_t* adc_value);
float CalculateBatteryVoltage(uint16_t* adc_value);
uint8_t GetBatteryPercentage(float* voltage);
float CalculateCurrent(uint16_t* adc_value);
uint16_t GetRPM(void);
void SensorsInit(volatile uint32_t* cnt,uint16_t timer_period);

#endif /* INC_SENSORS_H_ */
