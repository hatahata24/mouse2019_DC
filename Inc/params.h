#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

//a基本マクロ
#define R 0
#define L 1


//a機体情報
#define DIAMETER 24.5//25//24.5
#define TREAD 66

//PID パラメータ
#define Kp 3
#define Ti 1000
#define Td 0
#define Kp_o 3
#define Ti_o 1000
#define Td_o 0

//====a走行系パラメータ====
#define SEC_HALF 91//90.5
#define SEC_HALF_V 130
#define SEC_START 125
#define SEC_START_HALF 35.5

//a走行速度
#define SPEED_MIN 50
#define SPEED_LOW 400
#define SPEED_MIDDLE 600
#define SPEED_HIGH 800
#define SPEED_HIGH_HIGH 1200

#define LOW 1
#define MIDDLE 2
#define HIGH 3


//aセットポジション距離
#define SETPOS_BACK 100
#define SETPOS_SET 35

//aスラロームパラメータ
#define SLALOM_OFFSET_F 25//16.5
#define SLALOM_OFFSET_B 25//16.5
#define SLALOM_DEGACCEL 4000
#define SLALOM_OMEGA 550
#define SLALOM_DEG (SLALOM_OMEGA*SLALOM_OMEGA/SLALOM_DEGACCEL/2)

#define SLALOM_M_OFFSET_F 28
#define SLALOM_M_OFFSET_B 50//28
#define SLALOM_M_DEGACCEL 12000
#define SLALOM_M_OMEGA 900
#define SLALOM_M_DEG (SLALOM_H_OMEGA*SLALOM_H_OMEGA/SLALOM_H_DEGACCEL/2)

#define SLALOM_H_OFFSET_F 10//16.5
#define SLALOM_H_OFFSET_B 52//16.5
#define SLALOM_H_DEGACCEL 20000
#define SLALOM_H_OMEGA 800
#define SLALOM_H_DEG (SLALOM_H_OMEGA*SLALOM_H_OMEGA/SLALOM_H_DEGACCEL/2)

#define LSLALOM_OFFSET_F 50//45
#define LSLALOM_OFFSET_B 50//45
#define LSLALOM_DEGACCEL 2000
#define LSLALOM_OMEGA 200
#define LSLALOM_DEG (LSLALOM_OMEGA*LSLALOM_OMEGA/LSLALOM_DEGACCEL/2)

#define LSLALOM_M_OFFSET_F 63
#define LSLALOM_M_OFFSET_B 63
#define LSLALOM_M_DEGACCEL 4000
#define LSLALOM_M_OMEGA 400
#define LSLALOM_M_DEG (LSLALOM_H_OMEGA*LSLALOM_H_OMEGA/LSLALOM_H_DEGACCEL/2)

#define LSLALOM_H_OFFSET_F 60//45
#define LSLALOM_H_OFFSET_B 60//45
#define LSLALOM_H_DEGACCEL 4500
#define LSLALOM_H_OMEGA 600
#define LSLALOM_H_DEG (LSLALOM_H_OMEGA*LSLALOM_H_OMEGA/LSLALOM_H_DEGACCEL/2)

#define LSLALOM_H_H_OFFSET_F 42
#define LSLALOM_H_H_OFFSET_B 42
#define LSLALOM_H_H_DEGACCEL 10000
#define LSLALOM_H_H_OMEGA 800
#define LSLALOM_H_H_DEG (LSLALOM_H_H_OMEGA*LSLALOM_H_H_OMEGA/LSLALOM_H_H_DEGACCEL/2)

//a大回り180パラメータ
#define LROTATE_OFFSET_F 60
#define LROTATE_OFFSET_B 60
#define LROTATE_DEGACCEL 2000
#define LROTATE_OMEGA 260
#define LROTATE_DEG (LROTATE_OMEGA*LROTATE_OMEGA/LROTATE_DEGACCEL/2)

#define LROTATE_M_OFFSET_F 60
#define LROTATE_M_OFFSET_B 60
#define LROTATE_M_DEGACCEL 3000
#define LROTATE_M_OMEGA 600
#define LROTATE_M_DEG (LROTATE_H_OMEGA*LROTATE_H_OMEGA/LROTATE_H_DEGACCEL/2)

#define LROTATE_H_OFFSET_F 30//28
#define LROTATE_H_OFFSET_B 30//28
#define LROTATE_H_DEGACCEL 3000
#define LROTATE_H_OMEGA 600
#define LROTATE_H_DEG (LROTATE_H_OMEGA*LROTATE_H_OMEGA/LROTATE_H_DEGACCEL/2)

//a斜め走行パラメータ
#define V_OFFSET_F 40
#define V_OFFSET_B 80//75
#define V_DEGACCEL 3000
#define V_OMEGA 300
#define V_DEG (V_OMEGA*V_OMEGA/V_DEGACCEL/2)

#define V_M_OFFSET_F 40
#define V_M_OFFSET_B 76
#define V_M_DEGACCEL 8000
#define V_M_OMEGA 400
#define V_M_DEG (V_H_OMEGA*V_H_OMEGA/V_H_DEGACCEL/2)

#define V_H_OFFSET_F 23
#define V_H_OFFSET_B 80//60
#define V_H_DEGACCEL 8000
#define V_H_OMEGA 400
#define V_H_DEG (V_H_OMEGA*V_H_OMEGA/V_H_DEGACCEL/2)

#define VV_OFFSET_F 50//45//40
#define VV_OFFSET_B 50//45//40
#define VV_DEGACCEL 3500
#define VV_OMEGA 350
#define VV_DEG (VV_OMEGA*VV_OMEGA/VV_DEGACCEL/2)

#define VV_M_OFFSET_F 58
#define VV_M_OFFSET_B 58
#define VV_M_DEGACCEL 10000
#define VV_M_OMEGA 800
#define VV_M_DEG (VV_H_OMEGA*VV_H_OMEGA/VV_H_DEGACCEL/2)

#define VV_H_OFFSET_F 35
#define VV_H_OFFSET_B 65//35
#define VV_H_DEGACCEL 10000
#define VV_H_OMEGA 800
#define VV_H_DEG (VV_H_OMEGA*VV_H_OMEGA/VV_H_DEGACCEL/2)

#define VVV_OFFSET_F 70//65
#define VVV_OFFSET_B 58//48
#define VVV_DEGACCEL 3000
#define VVV_OMEGA 300
#define VVV_DEG (VVV_OMEGA*VVV_OMEGA/VVV_DEGACCEL/2)

#define VVV_M_OFFSET_F 70
#define VVV_M_OFFSET_B 54
#define VVV_M_DEGACCEL 5000
#define VVV_M_OMEGA 500
#define VVV_M_DEG (VVV_H_OMEGA*VVV_H_OMEGA/VVV_H_DEGACCEL/2)

#define VVV_H_OFFSET_F 120//80
#define VVV_H_OFFSET_B 70//65
#define VVV_H_DEGACCEL 8000
#define VVV_H_OMEGA 800
#define VVV_H_DEG (VVV_H_OMEGA*VVV_H_OMEGA/VVV_H_DEGACCEL/2)



//log取り用配列数
#define log_allay 200

//gyro 制御パラメータ
#define CTRL_BASE_G		1				//G
#define CTRL_MAX_G		1000			//control value max
#define CTRL_CONT_G		40//20//10				//Proportional C


//sensor 系パラメータ
#define sensor_wait 2500//2000//3500

//----wall 判断----
#define WALL_BASE_FR 	90//100//70//30				//FR
#define WALL_BASE_FL 	200//230//80//75//80//70				//FL
#define WALL_BASE_R		120//55				//R
#define WALL_BASE_L 	140//90	     		//L

//----wall 制御判断----
#define CTRL_BASE_R		30//50				//R
#define CTRL_BASE_L		35//				//L
#define CTRL_MAX_W		1000			//control value max
#define CTRL_CONT_W		0.75F//0.5F			//Proportional C

//----a停止時前壁補正----
#define OFFSET_FWALL_R 600
#define OFFSET_FWALL_L 1200

//----aスラローム前壁補正----
#define SLALOM_WALL_FR 250//225
#define SLALOM_WALL_FL 500//450

#define SLALOM_M_WALL_FR 400//190
#define SLALOM_M_WALL_FL 800//380

#define SLALOM_H_WALL_FR 400//190
#define SLALOM_H_WALL_FL 800//380

#define LSLALOM_WALL_FR 100//225
#define LSLALOM_WALL_FL 200//450

#define LSLALOM_M_WALL_FR 100//225
#define LSLALOM_M_WALL_FL 200//450

#define LSLALOM_H_WALL_FR 100//225
#define LSLALOM_H_WALL_FL 200//450

#define NO_WALL 5000

//----斜め時柱補正----
#define BASE_FR 300
#define BASE_FL 600
#define CTRL_BASE_FR 200
#define CTRL_BASE_FL 400

//----a壁切れ後長さ----
#define W_DIST 70

//music パラメータ
#define DO 1046
#define LE 1174
#define MI 1318
#define FA 1396
#define SO 1567
#define LA 1760
#define SI 1975
#define DOO 2093


//full led パラメータ
#define NO 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define BLUEGREEN 4
#define PURPLE 5
#define YELLOW 6
#define WHITE 7


//----goal 座標---
#define GOAL_X 9	//7
#define GOAL_Y 6	//7

#endif /* INC_PARAMS_H_ */
