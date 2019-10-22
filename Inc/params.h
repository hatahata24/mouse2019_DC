#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

//a機体情報
#define DIAMETER 24.5//25//24.5
#define TREAD 66

//PID パラメータ
#define Kp 2.5
#define Ti 1000
#define Td 0

//====a走行系パラメータ====
#define SEC_HALF 90.5
#define SEC_HALF_V 127
#define SEC_START 125
#define SEC_START_HALF 35.5

//a走行速度
#define SPEED_RUN 400
#define SPEED_MIN 50
#define SPEED_HIGH 800
#define SPEED_HIGH_HIGH 1200

//aセットポジション距離
#define SETPOS_BACK 150
#define SETPOS_SET 35

//aスラロームパラメータ
#define SLALOM_OFFSET 21
#define SLALOM_DEGACCEL 4000
#define SLALOM_OMEGA 550
#define SLALOM_H_OFFSET 34
#define SLALOM_H_DEGACCEL 20000
#define SLALOM_H_OMEGA 800
#define LSLALOM_OFFSET 50
#define LSLALOM_DEGACCEL 2000
#define LSLALOM_OMEGA 300
//#define LSLALOM_OFFSET 5
//#define LSLALOM_DEGACCEL 2000
//#define LSLALOM_OMEGA 200
#define LSLALOM_H_OFFSET 20
#define LSLALOM_H_DEGACCEL 4500
#define LSLALOM_H_OMEGA 600
#define LSLALOM_H_H_OFFSET 17
#define LSLALOM_H_H_DEGACCEL 10000
#define LSLALOM_H_H_OMEGA 800
#define LROTATE_OFFSET 60
#define LROTATE_DEGACCEL 2000
#define LROTATE_OMEGA 350
#define LROTATE_H_OFFSET 28
#define LROTATE_H_DEGACCEL 3000
#define LROTATE_H_OMEGA 600

//a斜め走行パラメータ
#define V_OFFSET 35
#define V_DEGACCEL 3000
#define V_OMEGA 300
#define V_H_OFFSET 23
#define V_H_DEGACCEL 8000
#define V_H_OMEGA 400
#define VV_OFFSET 50
#define VV_DEGACCEL 4000
#define VV_OMEGA 400
#define VV_H_OFFSET 35
#define VV_H_DEGACCEL 10000
#define VV_H_OMEGA 800
#define VVV_OFFSET 50
#define VVV_DEGACCEL 3000
#define VVV_OMEGA 360
#define VVV_H_OFFSET 65
#define VVV_H_DEGACCEL 8000
#define VVV_H_OMEGA 800


//a壁切れパラメータ
#define W_DIST 70

//log取り用配列数
#define log_allay 200

//gyro 制御パラメータ
#define CTRL_BASE_G		1				//G
#define CTRL_MAX_G		1000			//control value max
#define CTRL_CONT_G		10				//Proportional C


//sensor 系パラメータ
#define sensor_wait 2500//2000//3500

//----wall 判断----
#define WALL_BASE_FR 	90//100//70//30				//FR
#define WALL_BASE_FL 	200//230//80//75//80//70				//FL
#define WALL_BASE_R		90//100//55				//R
#define WALL_BASE_L 	90//100//45	     		//L

//----wall 制御判断----
#define CTRL_BASE_R		25//50//25				//R
#define CTRL_BASE_L		50//100//50				//L
#define CTRL_MAX_W		1000			//control value max
#define CTRL_CONT_W		0.5F//0.1F//1//0.8F			//Proportional C


//music パラメータ
#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093

//----goal 座標---
#define GOAL_X 6	//7
#define GOAL_Y 15	//7

#endif /* INC_PARAMS_H_ */
