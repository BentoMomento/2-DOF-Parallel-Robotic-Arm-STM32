/*
 * motor.h
 *
 *  Created on: Aug 12, 2024
 *      Author: Bento
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include "main.h"
#include <math.h>

typedef struct{
	int16_t step;
	uint8_t position;
	bool boundary;
} Stepper;

typedef struct {
	double x;
	double y;
	uint8_t l0;
	uint8_t l1;
	uint8_t l2;
} Arm;

typedef struct{
	int16_t a1;
	int16_t a2;
} Angle;

void init_system();

void init_stepper(int16_t step, uint8_t position, bool boundary, Stepper *target);

void init_arm(double x, double y, uint8_t l0, uint8_t l1, uint8_t l2, Arm *target);

void shiftIn(uint8_t data1, uint8_t data2);

void rotate(int16_t newStep1, int16_t newStep2);

void get_step(int16_t *new_step1, int16_t *new_step2);

void zero();

void init_step_table(Arm *arm, Angle *step_table[51][41]);

void uart_to_motor(double new_x, double new_y);

void reset_boundary(bool boundary1, bool boundary2);
#endif /* INC_MOTOR_H_ */
