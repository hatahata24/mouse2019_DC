#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

//machine params
#define DIAMETER 23.5
#define TREAD 60

//PID params
#define Kp 4
#define Ti 1000
#define Td 0

//drive params


//===sensor params===
#define sensor_wait 3000

//----wall judge----
#define WALL_BASE_FR 	40				//FR
#define WALL_BASE_FL 	150				//FL
#define WALL_BASE_R		50				//R
#define WALL_BASE_L 	100     		//L

//----wall control judge----
#define CTRL_BASE_L		50				//R
#define CTRL_BASE_R		50				//L
#define CTRL_MAX		3500			//control value max
#define CTRL_CONT		0.4F			//Proportional C


//===music params===
#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093

//----goal point---
#define GOAL_X 7	//7
#define GOAL_Y 7	//7

#endif /* INC_PARAMS_H_ */
