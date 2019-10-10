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
#define SPEED_HIGH_HIGH 1200

#define SETPOS_BACK 150
#define SETPOS_SET 35

#define SLALOM_OFFSET 23

#define W_DIST 70

//log params
#define log_allay 200

//gyro controll params
#define CTRL_BASE_G		1				//G
#define CTRL_MAX_G		1000			//control value max
#define CTRL_CONT_G		1				//Proportional C


//sensor params
#define sensor_wait 2500//2000//3500

//----wall judge----
#define WALL_BASE_FR 	150//70//30				//FR
#define WALL_BASE_FL 	250//80//75//80//70				//FL
#define WALL_BASE_R		100//55				//R
#define WALL_BASE_L 	100//45	     		//L

//----wall control judge----
#define CTRL_BASE_R		25				//R
#define CTRL_BASE_L		50				//L
#define CTRL_MAX_W		1000			//control value max
#define CTRL_CONT_W		0.5F//0.1F//1//0.8F			//Proportional C


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
