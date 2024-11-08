/*
 * motor.c
 *
 *  Created on: Aug 12, 2024
 *      Author: Bento
 */
#include "motor.h"

Arm arm;
Stepper motor1;
Stepper motor2;
Angle step_table[51][41]; // [x][y][motor#] x = 15, y = 12

void init_system(){
	// initialize 2 motor, and size of the motor
	init_arm(0, 0, 10, 10, 15, &arm);
	init_stepper(0, 0b1001, 1, &motor1);
	init_stepper(0, 0b1001, 1, &motor2);

	//construct Look up table
}

void init_stepper(int16_t step, uint8_t position, bool boundary, Stepper *target){
	target->step = 0;
	target->position = position;
	target->boundary = boundary;
}

void init_arm(double x, double y, uint8_t l0, uint8_t l1, uint8_t l2, Arm *target){
	target->x = x;
	target->y = y;
	target->l0 = l0;
	target->l1 = l1;
	target->l2 = l2;
}

void shiftIn(uint8_t data1, uint8_t data2){
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET); // PA12 = shift reg
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_RESET);  // PAA11 = storage reg

	 uint8_t data = 0b00000000;
	 data = data1<<4;
	 data = data|data2;

	 for(int8_t i=7;i>=0;i--){
		 //Sent data bit to DS pin
		 if((data&(0x01<<i)) != 0){
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET); // SET DS as 1
		 }
		 else{
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);  //SET DS as 0
		 }

		 //Delay 10 cycles
		 for(int8_t j=10;j>0;j--){

		 }

		 //Tick shift in clock
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);

		 // Delay 10 cycle
		 for(int8_t j=10;j>0;j--){

		 }
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);
	 }

	 // Release all the output by pulling storage reg high
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_SET);

	 for(int8_t j=10;j>0;j--){
	 }

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_RESET);

}

void rotate(int16_t newStep1, int16_t newStep2){

    // While at least one of step1 or step2 is non-zero
    while ((motor1.step != newStep1) | (motor2.step != newStep2)) {
        // Process step1
        if (motor1.step > newStep1){
            motor1.position = (motor1.position >> 1) | (motor1.position << 3);  // Rotate right
            motor1.step--;
        } else if (motor1.step > newStep1) {
            motor1.position = (motor1.position << 1) | (motor1.position >> 3);  // Rotate left
            motor1.step++;
        }

        // Process step2
        if (motor2.step > newStep2) {
            motor2.position = (motor2.position >> 1) | (motor2.position << 3);  // Rotate right
            motor2.step--;
        } else if (motor2.step > newStep2) {
            motor2.position = (motor2.position << 1) | (motor2.position >> 3);  // Rotate left
            motor2.step++;
        }

        // Mask to keep only the lower 4 bits of data1 and data2
        motor1.position &= 0b00001111;
        motor2.position &= 0b00001111;

        // Pass the modified data to the shiftIn function
        shiftIn(motor1.position, motor2.position);
    }
}

// will access the look up table and return the step position based on new x,y value located in Motor
// due to symetry, the LUT can be divided into 2 saving 50% of the storage
void get_step(int16_t *new_step1, int16_t *new_step2){

	uint8_t index_x = (uint8_t)(fabs(arm.x) / 0.3);
	uint8_t index_y = (uint8_t)arm.y / 0.3;
	if(arm.x > 0){
		*new_step1 = step_table[index_x][index_y].a1;
		*new_step2 = step_table[index_x][index_y].a2;
	}else{
		*new_step1 = 1024 - step_table[index_x][index_y].a2; // similar to pi - theta
		*new_step2 = step_table[index_x][index_y].a1;
	}
}

void zero(){

	uint8_t temp_step1 = 0;
	uint8_t temp_step2 = 0;

	// loop will terminate after both 2 EXIT receive interrupted by hall sensor
	while(motor1.boundary || motor2.boundary){

		if(motor1.boundary){
			temp_step1 = (motor1.step) + 1;
		}

		if(motor2.boundary){
			temp_step2 = (motor2.step) - 1;
		}

		rotate(temp_step1, temp_step2);
	}

	init_stepper(1024, motor1.position, 0,  &motor1);
	init_stepper(0, motor2.position, 0, &motor2);
}

void init_step_table(Arm *arm, Angle *step_table[51][41]){
    for (int i = 0; i < 51; i++) {
        double x = i * 0.3; // Step size of 0.3 for x

        for (int j = 0; j < 41; j++) {
            double y = j * 0.3; // Step size of 0.3 for y

            double alpha1, alpha2, beta1, beta2, d1, d2;

            d1 = (arm->l0 + x) * (arm->l0 + x) + y * y;
            d2 = (arm->l0 - x) * (arm->l0 - x) + y * y;

            alpha1 = acos((arm->l1 * arm->l1 + d1 - arm->l2 * arm->l2) / (2 * arm->l1 * sqrt(d1)));
            alpha2 = acos((arm->l1 * arm->l1 + d2 - arm->l2 * arm->l2) / (2 * arm->l1 * sqrt(d2)));

            beta1 = atan2(y, arm->l0 + x);
            beta2 = atan2(y, arm->l0 - x);
            step_table[i][j]->a1 = (int16_t)((alpha1 + beta1) / 2048);
            step_table[i][j]->a2 = (int16_t)((3.1415 - alpha2 - beta2) / 2048);
        }
    }
}

void uart_to_motor(double new_x, double new_y){

	arm.x = new_x;
	arm.y = new_y;

	// calculate difference in steps need to take, then call rotate to move
	int16_t new_step1;
	int16_t new_step2;
	get_step(&new_step1, &new_step2);
	rotate(new_step1, new_step2);
}

void reset_boundary(bool boundary1, bool boundary2){
	if(!boundary1){
		motor1.boundary = 0;
	}

	if(!boundary2){
		motor2.boundary = 0;
	}
}
