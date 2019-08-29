#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

//machine params
#define DIAMETER 24.5//25//24.5
#define TREAD 66

//PID params
#define Kp 2.5
#define Ti 1000
#define Td 0

//drive params
#define SEC_HALF 90
#define SEC_START 125
#define SPEED_RUN 400
#define SPEED_MIN 100
#define SPEED_HIGH 800

//log params
#define log_allay 200

//gyro controll params
#define CTRL_BASE_G		1				//G
#define CTRL_MAX_G		4000//3500				//control value max
#define CTRL_CONT_G		400//4000			//Proportional C


//sensor params
#define sensor_wait 3000

//----wall judge----
#define WALL_BASE_FR 	40				//FR
#define WALL_BASE_FL 	150				//FL
#define WALL_BASE_R		30				//R
#define WALL_BASE_L 	50	     		//L

//----wall control judge----
#define CTRL_BASE_L		20				//R
#define CTRL_BASE_R		20				//L
#define CTRL_MAX_W		350				//control value max
#define CTRL_CONT_W		0.8F			//Proportional C


//music params
#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093

//----goal point---
#define GOAL_X 9	//7
#define GOAL_Y 6	//7

#endif /* INC_PARAMS_H_ */
