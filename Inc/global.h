#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_


#include "main.h"

#include "params.h"
#include "sensor.h"
#include "search.h"
#include "buzzer.h"
#include "drive.h"
#include "gyro.h"
#include "eeprom.h"
#include <stdio.h>


typedef union {					//
	uint16_t FLAGS;
	struct ms_flags{			//
		uint16_t RSV0:1;		//a予備 bit(B0)		(:1は1ビット分の意味，ビットフィールド)
		uint16_t DRV:1;			//aモータ駆動 flag(B1)
		uint16_t SPD:1;			//a速度計算 flag(B2)
		uint16_t WCTRL:1;		//a壁制御 flag(B3)
		uint16_t GCTRL:1;		//aジャイロ制御 flag(B4)
		uint16_t SCND:1;		//a二次走行 flag(B5)
		uint16_t LOG:1;			//aログ flag(B6)
		uint16_t FWALL:1;		//a停止時前壁減製 flag(B7)
		uint16_t GYRO:1;		//a旋回角速度計算 flag(B8)
		uint16_t ACCL2:1;		//a既知区間加速 flag(B9)
		uint16_t STRAIGHT:1;	//a直線優先 flag(B10)
		uint16_t WEDGE:1;		//a壁切れフラグ(B11)
		uint16_t XDIR:1;		//8方向移動フラグ(B12)
		uint16_t RSV13:1;		//予備ビット(B13)
		uint16_t RSV14:1;		//予備ビット(B14)
		uint16_t RSV15:1;		//予備ビット(B15)
	}FLAG;
} mouse_flags;

#ifdef MAIN_C_							//main.cからこのファイルが呼ばれている場合
	/*グローバル変数の定義*/
	volatile mouse_flags MF;			//mouse kyoyou kouzoutai
#else									//main.c以外からこのファイルが呼ばれている場合
	/*グローバル変数の宣言*/
	extern volatile mouse_flags MF;
#endif

#endif /* INC_GLOBAL_H_ */
